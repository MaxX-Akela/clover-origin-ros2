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
