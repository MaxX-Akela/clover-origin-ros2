# shellcheck shell=bash
#
# Common helpers for the Clover image build scripts (sourced, not executed).
#
# Part of the ROS 2 port of Clover (https://github.com/CopterExpress/clover).
# echo_stamp() is taken from the original builder scripts,
# Copyright (C) 2018 Copter Express Technologies, Author: Artem Smirnov <urpylka@gmail.com>
#
# Distributed under MIT License (available at https://opensource.org/licenses/MIT).
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#

# Sourced by many scripts: define everything once
if [[ -n ${CLOVER_COMMON_LOADED:-} ]]; then
  return 0
fi
CLOVER_COMMON_LOADED=1

# Exit code of every script that refuses to run outside of the image build.
readonly GUARD_EXIT=64

echo_stamp() {
  # TEMPLATE: echo_stamp <TEXT> <TYPE>
  # TYPE: SUCCESS, ERROR, INFO
  local text color
  if [[ ${2:-INFO} != ERROR ]]; then
    CLOVER_LAST_STEP=$1
  fi
  text="$(date '+[%Y-%m-%d %H:%M:%S]') $1"
  case "${2:-INFO}" in
    SUCCESS) color=32;; # GREEN
    ERROR) color=31;;   # RED
    *) color=34;;       # BLUE
  esac
  printf '\e[1m\e[%sm%s\e[0m\n' "$color" "$text"
}

die() {
  echo_stamp "$*" ERROR >&2
  exit 1
}

retry() {
  # TEMPLATE: retry <COMMAND...>; RETRY_MAX attempts (default 3)
  local max=${RETRY_MAX:-3} count=1
  until "$@"; do
    if ((count >= max)); then
      echo_stamp "The command \"$*\" failed $max times" ERROR >&2
      return 1
    fi
    echo_stamp "The command \"$*\" failed. Retrying, $count of $max" ERROR >&2
    count=$((count + 1))
    sleep 1
  done
}

# True if $1 >= $2 in the sense of dpkg version comparison.
version_ge() {
  dpkg --compare-versions "$1" ge "$2"
}

is_wsl() {
  grep -qi microsoft /proc/version 2>/dev/null
}

# ---------------------------------------------------------------------------
# Protection against running image scripts on a developer machine.
#
# Scripts that attach loop devices, mount or chroot into an image start with
#   guard_init "$@"; set -- "${GUARD_ARGS[@]}"; require_ci
# Scripts that run INSIDE the image chroot call require_chroot instead.
# ---------------------------------------------------------------------------

# Strips --i-know from the arguments. Sets I_KNOW and GUARD_ARGS.
guard_init() {
  I_KNOW=0
  GUARD_ARGS=()
  local arg
  for arg in "$@"; do
    if [[ $arg == --i-know ]]; then
      I_KNOW=1
    else
      GUARD_ARGS+=("$arg")
    fi
  done
}

guard_refuse() {
  echo_stamp "Refusing to run $(basename "${0:-script}"): $*" ERROR >&2
  echo_stamp "This script mounts or modifies system images. Run it in CI (CI=true) or pass --i-know." ERROR >&2
  exit "$GUARD_EXIT"
}

require_ci() {
  if [[ ${I_KNOW:-0} == 1 ]]; then
    :
  elif [[ ${CI:-} == true ]]; then
    # CI=true is sometimes exported in an interactive shell, so under WSL it is not enough
    if is_wsl; then
      guard_refuse "CI=true is not accepted under WSL, pass --i-know"
    fi
  else
    guard_refuse "CI is not 'true' and --i-know is not given"
  fi
  if [[ $EUID -ne 0 ]]; then
    die "Root is required (run with sudo)"
  fi
}

readonly CHROOT_MARKER=/etc/clover_image_build

# Scripts executed inside the image chroot. The marker is created by
# image-chroot.sh in the mounted root, together with a token that is also
# passed in the environment, so CI=true or --i-know alone is not enough.
require_chroot() {
  local token=
  if [[ -f $CHROOT_MARKER ]]; then
    token=$(cat "$CHROOT_MARKER")
  fi
  if [[ -z $token || ${CLOVER_CHROOT_TOKEN:-} != "$token" ]]; then
    guard_refuse "not inside the image chroot (use image-chroot.sh exec)"
  fi
  trap chroot_failure_diag EXIT
}

# For scripts that are useful both on a CI runner and inside the chroot
# (they change apt sources of the system they run in).
require_ci_or_chroot() {
  if [[ -f $CHROOT_MARKER ]]; then
    require_chroot
  else
    require_ci
  fi
}

# ---------------------------------------------------------------------------
# Loop devices
# ---------------------------------------------------------------------------

LOOPDEV=

loop_attach() {
  # TEMPLATE: loop_attach <IMAGE>; sets LOOPDEV and waits for the partition nodes
  local i
  LOOPDEV=$(losetup --find --show --partscan "$1")
  for ((i = 0; i < 50; i++)); do
    if [[ -b ${LOOPDEV}p2 ]]; then
      return 0
    fi
    sleep 0.2
  done
  losetup -d "$LOOPDEV" || true
  LOOPDEV=
  die "Partition nodes of $1 did not appear"
}

loop_detach() {
  if [[ -n $LOOPDEV ]]; then
    sync
    losetup -d "$LOOPDEV" || echo_stamp "Could not detach $LOOPDEV" ERROR >&2
    LOOPDEV=
  fi
}

# ---------------------------------------------------------------------------
# apt inside the image (used by the chroot stages)
# ---------------------------------------------------------------------------

# Prints the apt sources, the policy of the given packages and the state of dpkg. Never fails.
# TEMPLATE: apt_diag [PACKAGES...]
# shellcheck disable=SC2119,SC2120
apt_diag() {
  local f
  echo_stamp "apt diagnostics"
  for f in /etc/apt/sources.list /etc/apt/sources.list.d/*; do
    if [[ -f $f ]]; then
      echo "--- $f"
      cat "$f" || true
    fi
  done
  if [[ $# -gt 0 ]]; then
    echo "--- apt-cache policy $*"
    apt-cache policy "$@" || true
  fi
  echo "--- apt-mark showhold"
  apt-mark showhold || true
  echo "--- apt-get check"
  apt-get check || true
  echo "--- dpkg --audit"
  dpkg --audit || true
}

# True if the suite is configured in one of the files (deb822 "Suites:" or one-line "deb ... suite ...").
# TEMPLATE: suite_configured <SUITE> <FILES...>
suite_configured() {
  local suite=$1 f
  shift
  for f in "$@"; do
    [[ -f $f ]] || continue
    if grep -Eq "^Suites:.*[[:space:]]${suite}([[:space:]]|\$)" "$f" || grep -Eq "^deb[[:space:]].*[[:space:]]${suite}[[:space:]]" "$f"; then
      return 0
    fi
  done
  return 1
}

# Adds <SUITE> to the first stanza of a deb822 file that has the plain codename in "Suites:".
# Returns 1 if nothing was changed.
# TEMPLATE: add_suite_to_stanza <FILE> <CODENAME> <SUITE>
add_suite_to_stanza() {
  local file=$1 codename=$2 suite=$3 tmp
  tmp=$(mktemp)
  awk -v codename="$codename" -v suite="$suite" '
    function flush(   i) {
      if (n > 0) {
        if (has_codename && !done) {
          for (i = 1; i <= n; i++) if (line[i] ~ /^Suites:/) line[i] = line[i] " " suite
          done = 1
        }
        for (i = 1; i <= n; i++) print line[i]
      }
      n = 0; has_codename = 0
    }
    /^[[:space:]]*$/ { flush(); print; next }
    {
      line[++n] = $0
      if ($0 ~ /^Suites:/) {
        for (i = 2; i <= NF; i++) if ($i == codename) has_codename = 1
      }
    }
    END { flush() }
  ' "$file" > "$tmp"
  if cmp -s "$file" "$tmp"; then
    rm -f "$tmp"
    return 1
  fi
  cat "$tmp" > "$file"
  rm -f "$tmp"
}

# Makes sure <CODENAME>-updates and <CODENAME>-security are configured (the Ubuntu Raspberry Pi
# image may come without them). The mirror for missing suites is ports.ubuntu.com/ubuntu-ports.
# TEMPLATE: ensure_ubuntu_suites <SOURCES_D> <SOURCES_LIST> <CODENAME>
ensure_ubuntu_suites() {
  local dir=$1 list=$2 codename=$3 files=() f missing=() suite
  for f in "$dir"/*.sources; do
    if [[ -f $f ]]; then
      files+=("$f")
    fi
  done
  files+=("$list")
  suite_configured "$codename" "${files[@]}" || die "Suite '${codename}' is not configured in ${dir} or ${list}"

  if ! suite_configured "${codename}-updates" "${files[@]}"; then
    for f in "${files[@]}"; do
      if [[ $f == *.sources ]] && add_suite_to_stanza "$f" "$codename" "${codename}-updates"; then
        echo_stamp "Added ${codename}-updates to ${f}"
        break
      fi
    done
  fi
  for suite in "${codename}-updates" "${codename}-security"; do
    suite_configured "$suite" "${files[@]}" || missing+=("$suite")
  done
  if [[ ${#missing[@]} -gt 0 ]]; then
    mkdir -p "$dir"
    cat > "${dir}/clover-ubuntu-suites.sources" << EOF
Types: deb
URIs: http://ports.ubuntu.com/ubuntu-ports
Suites: ${missing[*]}
Components: main restricted universe multiverse
Signed-By: /usr/share/keyrings/ubuntu-archive-keyring.gpg
EOF
    echo_stamp "Added ${missing[*]} in ${dir}/clover-ubuntu-suites.sources"
  fi
}

# apt-get update, then upgrade of everything except the linux-* packages: the kernel and its
# hooks (flash-kernel, initramfs) are not updated in a chroot. CLOVER_APT_DIST_UPGRADE=1 uses
# dist-upgrade. The list of changed packages is printed.
apt_upgrade() {
  local before after pkg rc=0 held=() action=upgrade
  if [[ ${CLOVER_APT_DIST_UPGRADE:-0} == 1 ]]; then
    action=dist-upgrade
  fi
  retry apt-get update
  before=$(mktemp)
  after=$(mktemp)
  dpkg-query -W -f='${Package} ${Version}\n' | sort > "$before"
  while IFS= read -r pkg; do
    held+=("$pkg")
  done < <(comm -23 \
    <(dpkg-query -W -f='${db:Status-Abbrev} ${Package}\n' | awk '$1 == "ii" && $2 ~ /^linux-/ {print $2}' | sort) \
    <(apt-mark showhold | sort))
  if [[ ${#held[@]} -gt 0 ]]; then
    echo_stamp "Hold for the upgrade: ${held[*]}"
    apt-mark hold "${held[@]}" > /dev/null
  fi
  echo_stamp "apt-get ${action}"
  retry apt-get -y --no-install-recommends -o Dpkg::Options::=--force-confdef -o Dpkg::Options::=--force-confold "$action" || rc=$?
  if [[ ${#held[@]} -gt 0 ]]; then
    apt-mark unhold "${held[@]}" > /dev/null || true
  fi
  dpkg-query -W -f='${Package} ${Version}\n' | sort > "$after"
  echo "--- packages changed by ${action} (old < / new >)"
  diff "$before" "$after" || true
  rm -f "$before" "$after"
  if [[ $rc -ne 0 ]]; then
    apt_diag
    die "apt-get ${action} failed with code ${rc}"
  fi
}

# Packages that the image builds; the same set as in .github/workflows/ci.yml
CLOVER_COLCON_PACKAGES=(aruco_pose led_msgs roswww_static clover)
# Console output of colcon build (written by image-ros.sh, read by the failure report)
CLOVER_COLCON_CONSOLE=/var/tmp/clover-colcon-console.log

# Runs a command as the build user (directly if we are not root, e.g. in the dry-run tests).
# TEMPLATE: diag_as_user <USER> <COMMAND...>
diag_as_user() {
  local user=$1
  shift
  if [[ $EUID -eq 0 ]]; then
    runuser -u "$user" -- env "HOME=$(getent passwd "$user" | cut -d: -f6)" "USER=${user}" "$@"
  else
    "$@"
  fi
}

# Why did "colcon build" fail: all in the CI log, never fails.
# TEMPLATE: build_failure_report <WORKSPACE> <USER> <ROS_DISTRO>
build_failure_report() {
  local ws=$1 user=$2 distro=$3 latest pkg f shown=0 built=() missing=()
  set +e
  echo "--- colcon console (Failed / Summary)"
  if [[ -f $CLOVER_COLCON_CONSOLE ]]; then
    grep -E 'Failed +<<<|Aborted +<<<|Summary:|packages? (failed|aborted)|Killed' "$CLOVER_COLCON_CONSOLE"
  else
    echo "no ${CLOVER_COLCON_CONSOLE} (colcon was not started or the stage is not image-ros)"
  fi

  for pkg in "${CLOVER_COLCON_PACKAGES[@]}"; do
    if [[ -f ${ws}/install/${pkg}/share/ament_index/resource_index/packages/${pkg} ]]; then
      built+=("$pkg")
    else
      missing+=("$pkg")
    fi
  done
  echo "--- installed in ${ws}/install: ${built[*]:-<none>}"
  echo "--- NOT installed (failed or not reached): ${missing[*]:-<none>}"
  echo "--- ls ${ws}/install"
  ls "${ws}/install" 2>&1

  latest=$(readlink -f "${ws}/log/latest_build" 2> /dev/null)
  if [[ ! -d $latest ]]; then
    latest=$(find "${ws}/log" -maxdepth 1 -type d -name 'build_*' 2> /dev/null | sort | tail -n 1)
  fi
  if [[ -d $latest ]]; then
    for pkg in "${missing[@]}"; do
      f="${latest}/${pkg}/stdout_stderr.log"
      if [[ -f $f ]]; then
        echo "--- tail -n 80 ${f}"
        tail -n 80 "$f"
        shown=1
      fi
    done
    if [[ $shown -eq 0 ]]; then
      echo "--- no package log, tail -n 80 ${latest}/events.log"
      tail -n 80 "${latest}/events.log" 2>&1
    fi
    echo "--- OOM-killed compiler in the logs (grep Killed)"
    grep -rl 'Killed' "$latest" --include=stdout_stderr.log --include=stderr.log 2> /dev/null
  else
    echo "--- no colcon log in ${ws}/log"
  fi
  echo "--- kernel OOM messages (dmesg)"
  dmesg 2> /dev/null | grep -iE 'out of memory|killed process' | tail -n 5

  if command -v rosdep > /dev/null && [[ -d ${ws}/src ]]; then
    echo "--- rosdep check --from-paths src --ignore-src --rosdistro ${distro}"
    diag_as_user "$user" bash -c "[ -f /opt/ros/${distro}/setup.bash ] && . /opt/ros/${distro}/setup.bash; \
      cd '${ws}' && rosdep check --from-paths src --ignore-src --rosdistro ${distro}" 2>&1
  fi

  echo "--- versions"
  cmake --version 2>&1 | head -n 1
  g++ --version 2>&1 | head -n 1
  python3 --version 2>&1
  if command -v ros2 > /dev/null; then
    echo "ros2: $(ros2 --version 2>&1 | head -n 1)"
  else
    echo "ros2: not in PATH (needs setup.bash)"
  fi
  echo "--- free -h"
  free -h
  echo "--- nproc: $(nproc)"
  echo "--- df -h"
  df -h
  return 0
}

# EXIT trap of the chroot stages (installed by require_chroot): the state of the image on failure.
chroot_failure_diag() {
  local rc=$? ws=/home/pi/ros2_ws stage
  trap - EXIT
  set +e
  if [[ $rc -ne 0 && $rc -ne $GUARD_EXIT ]]; then
    stage=$(basename "$0" .sh)
    {
      echo "BUILD FAILED AT STAGE: ${stage}"
      echo_stamp "FAILED (code ${rc}) in $(basename "$0"), last step: ${CLOVER_LAST_STEP:-<none>}" ERROR
      build_failure_report "$ws" pi jazzy
      apt_diag
      echo "--- tail /var/log/apt/term.log"
      tail -n 40 /var/log/apt/term.log
      echo "--- tail /var/log/dpkg.log"
      tail -n 20 /var/log/dpkg.log
      echo "--- key units (enabled)"
      systemctl list-unit-files --state=enabled --no-pager 2>&1 | grep -E '^(NetworkManager|nginx|clover|ssh|avahi)'
      echo "--- journalctl (if the image has a persistent journal)"
      journalctl -D /var/log/journal --no-pager -n 20
    } >&2
  fi
  exit "$rc"
}
