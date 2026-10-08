#!/usr/bin/env bash

#
# Script for image validation (runs inside the image chroot).
# There is no systemd as PID 1 and no hardware here, so only files, packages, the ROS
# environment and the hardware-free tests are checked. See image/README.md for the list
# of what can be checked only on a Raspberry Pi.
#
# Copyright (C) 2018 Copter Express Technologies
#
# Author: Artem Smirnov <urpylka@gmail.com>
#
# Distributed under MIT License (available at https://opensource.org/licenses/MIT).
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=scripts/common.sh
source "${SCRIPT_DIR}/scripts/common.sh"

# shellcheck source=image-network.sh
source "${SCRIPT_DIR}/image-network.sh"

USER_NAME=pi
USER_HOME="/home/${USER_NAME}"
BOOT_DIR=/boot/firmware
MAVROS_MIN_VERSION=2.16.0
REQUIRED_PACKAGES=(clover aruco_pose led_msgs roswww_static)
FAILURES=0

as_user() {
  runuser -u "$USER_NAME" -- env "HOME=${USER_HOME}" "USER=${USER_NAME}" "LOGNAME=${USER_NAME}" "$@"
}

FAILED_CHECKS=()
UNIT_DIRS=(/etc/systemd/system /usr/lib/systemd/system /lib/systemd/system /run/systemd/system)

# TEMPLATE: [CHECK_DIAG=<FUNCTION>] check <DESCRIPTION> <COMMAND...>
# On a failure prints the description, the exact command and its output; CHECK_DIAG (optional)
# is a function that prints what was examined. The name of the check goes to the "last step"
# of the failure report of common.sh.
check() {
  local description=$1 output rc=0
  shift
  CLOVER_LAST_STEP="running check: ${description}"
  output=$("$@" 2>&1) || rc=$?
  if ((rc == 0)); then
    echo_stamp "ok: ${description}" SUCCESS
  else
    echo_stamp "FAILED: ${description}" ERROR
    {
      echo "  command: $*"
      echo "  exit code: ${rc}"
      if [[ -n $output ]]; then
        echo "  output:"
        tail -n 20 <<< "$output" | sed 's/^/    /'
      fi
      if [[ -n ${CHECK_DIAG:-} ]]; then
        echo "  diagnostics (${CHECK_DIAG}):"
        "$CHECK_DIAG" 2>&1 | sed 's/^/    /' || true
      fi
    } >&2
    FAILED_CHECKS+=("$description")
    FAILURES=$((FAILURES + 1))
  fi
  CLOVER_LAST_STEP="check: ${description}"
}

# Same, but only a warning
warn_check() {
  local description=$1
  shift
  CLOVER_LAST_STEP="running check: ${description}"
  if "$@" > /dev/null 2>&1; then
    echo_stamp "ok: ${description}" SUCCESS
  else
    echo_stamp "WARNING: ${description}" ERROR
  fi
  CLOVER_LAST_STEP="check: ${description}"
}

# TEMPLATE: unit_files <NAME> [ROOT]; prints the files of the unit (also dangling symlinks and
# *.wants/*.requires links). Files are looked for directly: in a chroot "systemctl cat" prints
# "Running in chroot, ignoring command" and exits 0 for any name, so it cannot be used here.
unit_files() {
  local name=$1 root=${2:-} dir
  for dir in "${UNIT_DIRS[@]}"; do
    [[ -d ${root}${dir} ]] || continue
    find "${root}${dir}" -maxdepth 2 \( -name "$name" -o -name "${name}.d" \) -print 2> /dev/null
  done
}

# TEMPLATE: check_no_unit <NAME> [ROOT]
check_no_unit() {
  local found
  found=$(unit_files "$1" "${2:-}")
  if [[ -n $found ]]; then
    echo "unit file(s) of $1 exist:"
    echo "$found"
    return 1
  fi
}

diag_roscore_unit() {
  local dir
  for dir in "${UNIT_DIRS[@]}"; do
    echo "--- ls -la ${dir}"
    # shellcheck disable=SC2012 # a listing for people, not for parsing
    ls -la "$dir" 2>&1 | head -n 60
    echo "--- grep -ril roscore ${dir}"
    grep -ril roscore "$dir" 2>&1 | head -n 20
  done
  echo "--- dpkg -S for the found files"
  unit_files roscore.service | while IFS= read -r f; do dpkg -S "$f" 2>&1; done
}

in_ros() {
  # TEMPLATE: in_ros <COMMAND...>; the ROS environment is sourced without nounset
  bash -c '. /etc/clover/ros-env.sh && "$@"' bash "$@"
}

mavros_version() {
  # apt package or the source workspace
  local version
  version=$(dpkg-query -W -f='${Version}' ros-jazzy-mavros 2> /dev/null || true)
  if [[ -z $version && -f /opt/mavros_ws/src/mavros/mavros/package.xml ]]; then
    version=$(sed -n 's:.*<version>\(.*\)</version>.*:\1:p' /opt/mavros_ws/src/mavros/mavros/package.xml | head -n 1)
  fi
  echo "$version"
}

check_mavros_version() {
  local version
  version=$(mavros_version)
  echo "mavros version: ${version:-none}"
  [[ -n $version ]] && version_ge "$version" "$MAVROS_MIN_VERSION"
}

# ROS_HOSTNAME and ROS_IP do not exist in ROS 2; comments may mention them
check_no_ros1_env() {
  ! grep -rhvE '^[[:space:]]*#' /etc/clover "${USER_HOME}/.bashrc" | grep -qE 'ROS_HOSTNAME|ROS_IP'
}

check_packages_listed() {
  local list pkg
  list=$(in_ros ros2 pkg list)
  for pkg in "${REQUIRED_PACKAGES[@]}"; do
    grep -qx "$pkg" <<< "$list" || { echo "missing ROS package: $pkg"; return 1; }
  done
}

check_ap_profile() {
  local file=${1:-/etc/NetworkManager/system-connections/clover-ap.nmconnection}
  [[ $(stat -c %a "$file") == 600 ]] || return 1
  python3 - "$file" << 'EOF'
import configparser
import sys

c = configparser.ConfigParser(interpolation=None)
c.read(sys.argv[1])
assert c['wifi']['mode'] == 'ap'
assert c['wifi-security']['psk'] == 'cloverwifi'
assert c['ipv4']['method'] == 'shared'
assert c['ipv4']['address1'] == '192.168.11.1/24'
EOF
}

# TEMPLATE: check_ap_profile_radio [FILE]; the profile must not depend on the country: 2.4 GHz with a channel
check_ap_profile_radio() {
  local file=${1:-/etc/NetworkManager/system-connections/clover-ap.nmconnection}
  python3 - "$file" << 'PYEND'
import configparser
import sys

c = configparser.ConfigParser(interpolation=None)
c.read(sys.argv[1])
assert c['connection']['id'] == 'clover-ap'
assert c['connection']['type'] == 'wifi'
assert c['connection']['autoconnect'] == 'true'
assert c['connection']['uuid']
assert c['wifi']['band'] == 'bg', 'band must be bg (2.4 GHz)'
assert 1 <= int(c['wifi']['channel']) <= 11, 'channel 1..11 is allowed in every regulatory domain'
assert c['wifi'].get('ssid')
# a fixed interface name would break the profile if the interface is not wlan0 (wlP..., wlx...)
assert 'interface-name' not in c['connection'], 'interface-name pins the profile to one device name'
PYEND
}

# TEMPLATE: check_wifi_regdom [CMDLINE_FILE]; a non-empty regulatory domain (two letters or 00)
check_wifi_regdom() {
  local cmdline=${1:-${BOOT_DIR}/cmdline.txt}
  grep -Eq '(^| )cfg80211\.ieee80211_regdom=([A-Z]{2}|00)( |$)' "$cmdline"
}

# TEMPLATE: check_wlan_managed [ROOT]; nothing makes NetworkManager ignore wlan0 / Wi-Fi, nothing renders it elsewhere
check_wlan_managed() {
  local root=${1:-} found
  found=$(grep -rsE 'unmanaged-devices|^[[:space:]]*wifis:|wlan0' \
    "${root}/etc/NetworkManager/conf.d" "${root}/usr/lib/NetworkManager/conf.d" "${root}/etc/NetworkManager/NetworkManager.conf" \
    "${root}/etc/netplan" "${root}/etc/cloud/cloud.cfg.d" "${root}/boot/firmware/network-config" "${root}/boot/firmware/user-data" \
    | grep -vE ':[[:space:]]*#' | grep -vE 'unmanaged-devices=none[[:space:]]*$' || true)
  if [[ -n $found ]]; then
    echo "Wi-Fi is configured or disabled outside of the clover-ap profile:"
    echo "$found"
    return 1
  fi
}

# TEMPLATE: check_no_saved_rfkill_block [ROOT]; systemd-rfkill restores the saved state (1 = blocked)
check_no_saved_rfkill_block() {
  local root=${1:-} f
  for f in "${root}"/var/lib/systemd/rfkill/*; do
    [[ -e $f ]] || continue
    [[ $(< "$f") == 0 ]] || { echo "saved rfkill state ${f} = $(< "$f") (blocked)"; return 1; }
  done
}

# TEMPLATE: check_firstboot_script [FILE]; the first boot unblocks the radio, sets the domain and brings the profile up
check_firstboot_script() {
  local file=${1:-/usr/local/sbin/clover-firstboot}
  [[ -x $file ]] || return 1
  grep -q 'rfkill unblock wifi' "$file" && grep -q 'iw reg set' "$file" && grep -q 'nmcli connection up clover-ap' "$file"
}

check_ap_prerequisites() {
  # NetworkManager starts dnsmasq itself for ipv4.method=shared; the system dnsmasq.service would take port 53
  command -v dnsmasq > /dev/null || { echo "dnsmasq (package dnsmasq-base) is missing"; return 1; }
  command -v rfkill > /dev/null || { echo "rfkill is missing"; return 1; }
  command -v iw > /dev/null || { echo "iw is missing"; return 1; }
  [[ -f /usr/sbin/wpa_supplicant ]] || { echo "wpa_supplicant is missing"; return 1; }
  [[ -f /usr/share/wireless-regdb/regulatory.db || -f /lib/firmware/regulatory.db ]] || { echo "regulatory.db is missing (wireless-regdb)"; return 1; }
  ! systemctl is-enabled dnsmasq.service 2> /dev/null | grep -qx enabled
}

diag_network() {
  show_network_state
}

check_boot_files() {
  local cmdline=${BOOT_DIR}/cmdline.txt config=${BOOT_DIR}/config.txt
  [[ $(wc -l < "$cmdline") -eq 1 ]] || return 1
  ! grep -q 'console=serial0' "$cmdline" || return 1
  grep -q '^\[pi4\]$' "$config" && grep -q '^\[pi5\]$' "$config" || return 1
  grep -q '^dtoverlay=disable-bt$' "$config" && grep -q '^dtoverlay=uart0-pi5$' "$config"
}

check_overlays() {
  [[ -f ${BOOT_DIR}/overlays/uart0-pi5.dtbo && -f ${BOOT_DIR}/overlays/disable-bt.dtbo ]]
}

show_dtb_aliases() {
  # Informational: names of the serial ports in the device trees of the image (see NOTES.md)
  local dtb
  for dtb in bcm2711-rpi-4-b.dtb bcm2712-rpi-5-b.dtb; do
    echo "--- ${dtb}: serial aliases"
    if [[ -f ${BOOT_DIR}/${dtb} ]]; then
      dtc -I dtb -O dts "${BOOT_DIR}/${dtb}" 2> /dev/null | sed -n '/aliases {/,/};/p' | grep -E 'serial|uart' || echo "(none)"
    else
      echo "${dtb} not found"
    fi
  done
}

check_dtbs_present() {
  [[ -f ${BOOT_DIR}/bcm2711-rpi-4-b.dtb && -f ${BOOT_DIR}/bcm2712-rpi-5-b.dtb ]]
}

show_libcamera_pipelines() {
  # Informational: does the libcamera of ROS know the Raspberry Pi pipelines (vc4 = Pi 4, pisp = Pi 5)
  local vc4=0 pisp=0 file
  while IFS= read -r file; do
    grep -aq 'vc4' "$file" && vc4=1
    grep -aq 'pisp' "$file" && pisp=1
  done < <(find /opt/ros/jazzy -type f \( -name 'libcamera*.so*' -o -path '*libcamera*' -name '*.so*' \) 2> /dev/null)
  echo "libcamera strings: vc4=${vc4} pisp=${pisp}"
  [[ $vc4 == 1 && $pisp == 1 ]]
}

run_hardware_free_tests() {
  # roswww_static has pure Python tests; the packages are built apart from the image build (BUILD_TESTING=OFF there)
  local tmp
  tmp=$(mktemp -d)
  chown "${USER_NAME}:${USER_NAME}" "$tmp"
  as_user bash -c "
    . /etc/clover/ros-env.sh
    cd ${USER_HOME}/ros2_ws
    colcon build --packages-select roswww_static --build-base ${tmp}/build --install-base ${tmp}/install \
      --cmake-args -DBUILD_TESTING=ON &&
    colcon test --packages-select roswww_static --build-base ${tmp}/build --install-base ${tmp}/install \
      --test-result-base ${tmp}/test_results --event-handlers console_direct+ &&
    colcon test-result --test-result-base ${tmp}/test_results --verbose
  "
  local rc=$?
  rm -rf "$tmp" "${USER_HOME}/ros2_ws/log/latest_test"
  return $rc
}

main() {
  require_chroot

  echo_stamp "Run image tests"
  check "version and origin files" bash -c '[[ -s /etc/clover_version && -s /etc/clover_origin ]]'
  check "user ${USER_NAME} is in dialout and sudo" bash -c "id -nG ${USER_NAME} | tr ' ' '\\n' | grep -qx dialout && id -nG ${USER_NAME} | tr ' ' '\\n' | grep -qx sudo"

  echo_stamp "ROS 2 and Clover packages"
  check "ROS packages ${REQUIRED_PACKAGES[*]} are listed" check_packages_listed
  check "ros2 interface show clover/srv/Navigate" in_ros ros2 interface show clover/srv/Navigate
  check "ros2 interface show led_msgs/srv/SetLEDs" in_ros ros2 interface show led_msgs/srv/SetLEDs
  check "python import of clover (service_proxy, long_callback, srv)" in_ros python3 -c \
    'from clover import service_proxy, long_callback; from clover.srv import Navigate'
  check "mavros >= ${MAVROS_MIN_VERSION}" check_mavros_version
  check "GeographicLib geoid egm96-5" test -f /usr/share/GeographicLib/geoids/egm96-5.pgm
  check "pymavlink" python3 -c 'import pymavlink'
  check "no ROS_HOSTNAME / ROS_IP in the environment files" check_no_ros1_env
  warn_check "libcamera of ROS has Raspberry Pi pipelines (vc4, pisp); CSI camera is not verified" show_libcamera_pipelines

  echo_stamp "Services and configuration files"
  check "clover.service is enabled" systemctl is-enabled clover.service
  check "clover-firstboot.service is enabled" systemctl is-enabled clover-firstboot.service
  check "NetworkManager, nginx, avahi-daemon are enabled" bash -c 'systemctl is-enabled NetworkManager.service nginx.service avahi-daemon.service'
  CHECK_DIAG=diag_roscore_unit check "no roscore.service" check_no_unit roscore.service
  warn_check "systemd-analyze verify of the units" systemd-analyze verify /etc/systemd/system/clover.service /etc/systemd/system/clover-firstboot.service
  check "nginx -t" nginx -t
  check "Wi-Fi access point profile" check_ap_profile
  check "access point profile: autoconnect, 2.4 GHz (bg) with a channel, no fixed interface name" check_ap_profile_radio
  CHECK_DIAG=diag_network check "access point prerequisites (dnsmasq, rfkill, iw, wpa_supplicant, regulatory.db, no system dnsmasq.service)" check_ap_prerequisites
  check "Wi-Fi regulatory domain in cmdline.txt" check_wifi_regdom
  CHECK_DIAG=diag_network check "wlan0 is not disabled or unmanaged by NetworkManager, netplan or cloud-init" check_wlan_managed
  check "no saved rfkill block" check_no_saved_rfkill_block
  check "clover-firstboot unblocks Wi-Fi, sets the domain, brings clover-ap up" check_firstboot_script
  check "udev rules of Clover are linked" bash -c 'ls /etc/udev/rules.d/ | grep -q px4fmu'
  check "examples symlink" test -d "${USER_HOME}/examples"
  check "/home/pi/.ros/www exists" test -d "${USER_HOME}/.ros/www"

  echo_stamp "Raspberry Pi 4 / 5 boot files"
  check "config.txt and cmdline.txt" check_boot_files
  check "overlays uart0-pi5.dtbo and disable-bt.dtbo" check_overlays
  check "device trees of Raspberry Pi 4 and 5" check_dtbs_present
  show_dtb_aliases || true

  echo_stamp "Hardware-free tests (colcon test roswww_static)"
  check "colcon test roswww_static" run_hardware_free_tests

  if ((FAILURES > 0)); then
    CLOVER_LAST_STEP="failed checks: $(printf '%s; ' "${FAILED_CHECKS[@]}")"
    die "${FAILURES} check(s) failed"
  fi
  echo_stamp "All checks passed (nothing was started on hardware)" SUCCESS
}

# Sourcing (for tests) defines the functions only
if [[ ${BASH_SOURCE[0]} == "$0" ]]; then
  main "$@"
fi
