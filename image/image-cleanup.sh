#!/usr/bin/env bash

#
# Script for the cleanup of the image (runs inside the image chroot).
#
#   image-cleanup.sh [packages|final|all]
#
#   packages  measure, remove what a drone does not need, clean apt, the workspace, docs and locales.
#             Runs BEFORE image-validate.sh, which checks the result (and needs the build scripts).
#   final     everything that must be created again on the first boot, logs, temporary files and
#             the build scripts. Runs AFTER image-validate.sh.
#   all       both (default)
#
# Every step is a function and prints "saved NNN MiB" (used space of the root filesystem).
# The cloud-init seed on /boot/firmware stays.
#
# Part of the ROS 2 port of Clover (https://github.com/CopterExpress/clover).
#
# Distributed under MIT License (available at https://opensource.org/licenses/MIT).
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=scripts/common.sh
source "${SCRIPT_DIR}/scripts/common.sh"

USER_NAME=pi
USER_HOME="/home/${USER_NAME}"
WS_DIR="${USER_HOME}/ros2_ws"

# Not needed on a drone. The list is only a request: see purge_unneeded_packages.
UNNEEDED_PACKAGES=(
  snapd
  lxd-installer
  open-iscsi
  multipath-tools
  open-vm-tools
  modemmanager
  unattended-upgrades
  ubuntu-pro-client
  ubuntu-advantage-tools
  fwupd
  apport
  popularity-contest
  landscape-common
  motd-news-config
  sysstat
)
# ufw stays: it is installed but inactive in the Ubuntu images, so it blocks neither the access point
# (DHCP 67, DNS 53 of NetworkManager) nor ssh; removing it saves about 1 MiB only.

# Packages that must survive every removal (regular expression for the whole package name)
PROTECTED_REGEX='^(network-manager|cloud-init|avahi-daemon|openssh-server|openssh-sftp-server|nginx(-.*)?|ros-jazzy-.*|ros-dev-tools|linux-.*|libcamera.*|netplan\.io|wpasupplicant|dnsmasq-base|nftables|iw|rfkill|wireless-regdb|systemd|systemd-sysv|dbus|python3|cloud-guest-utils|cloud-initramfs-growroot|e2fsprogs|util-linux|sudo)$'

# Timers that would take the apt lock after the boot
APT_TIMERS=(apt-daily.timer apt-daily-upgrade.timer apt-daily.service apt-daily-upgrade.service motd-news.timer motd-news.service)

# Used space of the root filesystem, MiB
used_mib() {
  df --output=used -B1M "${1:-/}" | tail -n 1 | tr -d ' '
}

# TEMPLATE: step <FUNCTION> [ARGS...]; runs the function and prints how much space it freed
step() {
  local fn=$1 before after
  shift
  echo_stamp "cleanup: ${fn}"
  before=$(used_mib /)
  "$fn" "$@"
  after=$(used_mib /)
  STEP_LOG+=("${fn}"$'\t'"$((before - after))")
  echo_stamp "cleanup: ${fn}: saved $((before - after)) MiB (used ${before} -> ${after} MiB)" SUCCESS
}
STEP_LOG=()

# --- measurement ------------------------------------------------------------------------------

measure_sizes() {
  echo "--- biggest packages (Installed-Size, KiB)"
  { dpkg-query -W -f='${Installed-Size}\t${Package}\n' | sort -rn | head -n 50; } || true
  echo "--- du -xh --max-depth=2 /"
  { du -xh --max-depth=2 / 2> /dev/null | sort -rh | head -n 40; } || true
  echo "--- df -h /"
  df -h / || true
}

# --- packages ---------------------------------------------------------------------------------

# TEMPLATE: protected_filter; reads package names on stdin, prints the protected ones
protected_filter() {
  grep -E "$PROTECTED_REGEX" || true
}

# TEMPLATE: simulated_removals; reads the output of "apt-get -s" on stdin, prints the names of removed packages
simulated_removals() {
  awk '/^Remv / {sub(/:[a-z0-9]+$/, "", $2); print $2}'
}

# TEMPLATE: installed_subset <NAMES...>; prints those that are installed
installed_subset() {
  local pkg
  for pkg in "$@"; do
    if dpkg-query -W -f='${db:Status-Abbrev}' "$pkg" 2> /dev/null | grep -q '^.i'; then
      echo "$pkg"
    fi
  done
}

# Protected packages are marked as manually installed: removal of a metapackage (ubuntu-server-raspi, which
# depends on snapd, lxd-installer, open-iscsi...) must not drag them into autoremove.
mark_protected_manual() {
  local names
  names=$(dpkg-query -W -f='${binary:Package}\n' | sed 's/:.*//' | protected_filter)
  if [[ -n $names ]]; then
    # shellcheck disable=SC2086 # one word per package
    apt-mark manual $names > /dev/null
  fi
}

# TEMPLATE: safe_to_remove <PACKAGE> [ROOT_APT_ARGS...]; true if "apt-get -s purge" removes no protected package
safe_to_remove() {
  local pkg=$1 removed protected
  removed=$(apt-get -s purge "$pkg" 2> /dev/null | simulated_removals) || return 1
  protected=$(protected_filter <<< "$removed")
  if [[ -n $protected ]]; then
    echo "NOT removing ${pkg}: would remove protected package(s): $(tr '\n' ' ' <<< "$protected")"
    return 1
  fi
}

purge_unneeded_packages() {
  local installed pkg safe=()
  mapfile -t installed < <(installed_subset "${UNNEEDED_PACKAGES[@]}")
  if ((${#installed[@]} == 0)); then
    echo "none of the unneeded packages is installed"
    return 0
  fi
  mark_protected_manual
  for pkg in "${installed[@]}"; do
    if safe_to_remove "$pkg"; then
      safe+=("$pkg")
    fi
  done
  if ((${#safe[@]} == 0)); then
    return 0
  fi
  # all together must be safe as well
  local all_removed protected
  all_removed=$(apt-get -s purge "${safe[@]}" | simulated_removals)
  protected=$(protected_filter <<< "$all_removed")
  [[ -z $protected ]] || die "purge of ${safe[*]} would remove protected packages: ${protected}"
  echo "purging: ${safe[*]}"
  echo "all packages removed by the purge: $(tr '\n' ' ' <<< "$all_removed")"
  apt-get purge -y "${safe[@]}"
}

remove_snap_leftovers() {
  rm -rf /snap /var/snap /var/lib/snapd /var/cache/snapd "${USER_HOME}/snap" /root/snap
  # snapd must not come back with an update or as a dependency (hold works only for installed packages)
  install -D -m 644 /dev/stdin /etc/apt/preferences.d/clover-nosnap << 'EOF'
Package: snapd
Pin: release a=*
Pin-Priority: -10
EOF
}

disable_apt_timers() {
  local unit
  for unit in "${APT_TIMERS[@]}"; do
    systemctl mask "$unit" > /dev/null 2>&1 || echo "could not mask ${unit} (no such unit?)"
  done
}

autoremove_unused() {
  local removed protected
  removed=$(apt-get -s autoremove --purge | simulated_removals)
  protected=$(protected_filter <<< "$removed")
  if [[ -n $protected ]]; then
    echo "WARNING: autoremove skipped, it would remove protected package(s): $(tr '\n' ' ' <<< "$protected")"
    return 0
  fi
  echo "autoremove: $(tr '\n' ' ' <<< "$removed")"
  apt-get autoremove --purge -y
}

clean_apt() {
  apt-get clean -qq
  rm -rf /var/lib/apt/lists/* /var/cache/apt/* /var/cache/debconf/*-old
}

# --- workspace, docs, locales -------------------------------------------------------------------

# TEMPLATE: install_links_into_build <INSTALL_DIR> <BUILD_DIR>; prints symlinks of the install space that point into build
install_links_into_build() {
  local install=$1 build=$2 link
  [[ -d $install && -d $build ]] || return 0
  build=$(realpath "$build")
  while IFS= read -r link; do
    case "$(readlink -f "$link")" in
      "$build"/*) echo "$link";;
    esac
  done < <(find "$install" -type l)
}

# colcon --symlink-install links the files generated at build time (messages in Python, ...) into build/.
# build/ is removed only when the install space does not use it.
clean_workspace() {
  local ws=${1:-$WS_DIR} used
  rm -rf "${ws}/log"
  if [[ -d ${ws}/build ]]; then
    used=$(install_links_into_build "${ws}/install" "${ws}/build" | head -n 5)
    if [[ -n $used ]]; then
      echo "WARNING: build/ is kept, install/ links into it, for example:"
      echo "$used"
    else
      rm -rf "${ws}/build"
    fi
  fi
  find "${ws}/src" -name __pycache__ -type d -prune -exec rm -rf {} + 2> /dev/null || true
}

# TEMPLATE: clean_docs [ROOT]; documentation, man pages, lintian; licenses (copyright) stay
clean_docs() {
  local root=${1:-}
  if [[ -d ${root}/usr/share/doc ]]; then
    find "${root}/usr/share/doc" -xdev -type f ! -name copyright -delete
    find "${root}/usr/share/doc" -xdev -depth -type d -empty -delete
    # links to deleted files
    find "${root}/usr/share/doc" -xdev -xtype l -delete
  fi
  rm -rf "${root}"/usr/share/man/* "${root}"/usr/share/info/* "${root}"/usr/share/lintian/* "${root}"/usr/share/groff/* 2> /dev/null || true
}

# TEMPLATE: clean_locales [ROOT]; only en and ru stay
clean_locales() {
  local root=${1:-}
  [[ -d ${root}/usr/share/locale ]] || return 0
  find "${root}/usr/share/locale" -mindepth 1 -maxdepth 1 ! -name 'en*' ! -name 'ru*' ! -name locale.alias -exec rm -rf {} +
}

# dpkg does not unpack the removed files again after an update on the board
dpkg_exclude_docs() {
  install -D -m 644 /dev/stdin /etc/dpkg/dpkg.cfg.d/90-clover-nodoc << 'EOF'
path-exclude=/usr/share/doc/*
path-include=/usr/share/doc/*/copyright
path-exclude=/usr/share/man/*
path-exclude=/usr/share/info/*
path-exclude=/usr/share/lintian/*
path-exclude=/usr/share/locale/*
path-include=/usr/share/locale/en*/*
path-include=/usr/share/locale/ru*/*
path-include=/usr/share/locale/locale.alias
EOF
}

# --- final stage ----------------------------------------------------------------------------------

# TEMPLATE: truncate_logs [ROOT]; keeps the directories and the files, empties the content, removes rotated logs
truncate_logs() {
  local root=${1:-}
  [[ -d ${root}/var/log ]] || return 0
  find "${root}/var/log" -type f \( -name '*.gz' -o -name '*.[0-9]' -o -name '*.old' \) -delete
  find "${root}/var/log" -type f -exec truncate -s 0 {} +
  rm -rf "${root}"/var/log/journal/*
}

remove_host_state() {
  # Regenerated on the first boot: ssh host keys by cloud-init, machine-id by systemd
  rm -f /etc/ssh/ssh_host_*
  : > /etc/machine-id
  rm -rf /var/lib/cloud/instances /var/lib/cloud/instance /var/lib/cloud/data/*
}

remove_caches() {
  rm -rf /root/.cache /root/.ros /root/.bash_history "${USER_HOME}/.cache" "${USER_HOME}/.bash_history"
  rm -rf "${WS_DIR}/log"
  rm -rf /tmp/* /var/tmp/*
}

remove_build_scripts() {
  rm -rf /root/clover-image
}

# --- stages -----------------------------------------------------------------------------------------

stage_packages() {
  echo_stamp "Measurement before the cleanup (the real big items, not guesses)"
  measure_sizes
  step purge_unneeded_packages
  step remove_snap_leftovers
  step disable_apt_timers
  step autoremove_unused
  step clean_apt
  step clean_workspace
  step clean_docs
  step clean_locales
  step dpkg_exclude_docs
  echo_stamp "Measurement after the cleanup"
  measure_sizes
}

stage_final() {
  step truncate_logs
  step remove_host_state
  step remove_caches
  step clean_apt
  step remove_build_scripts
}

print_summary() {
  local line
  echo_stamp "Cleanup summary: step -> freed MiB"
  for line in "${STEP_LOG[@]}"; do
    printf '  %-28s %6s MiB\n' "${line%%$'\t'*}" "${line#*$'\t'}"
  done
  df -h / || true
}

main() {
  require_chroot
  local stage=${1:-all}
  case "$stage" in
    packages) stage_packages;;
    final) stage_final;;
    all) stage_packages; stage_final;;
    *) die "Usage: $0 [packages|final|all]";;
  esac
  print_summary
  echo_stamp "End of cleanup (${stage})" SUCCESS
}

# Sourcing (for tests) defines the functions only
if [[ ${BASH_SOURCE[0]} == "$0" ]]; then
  main "$@"
fi
