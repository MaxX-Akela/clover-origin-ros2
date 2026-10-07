#!/usr/bin/env bash

#
# Script for hardware interfaces of Raspberry Pi 4 and Raspberry Pi 5 (runs inside the image chroot):
# /boot/firmware/config.txt, /boot/firmware/cmdline.txt, UART, I2C, SPI, udev.
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

ASSETS_DIR="${SCRIPT_DIR}/assets"
BOOT_DIR=${CLOVER_BOOT_DIR:-/boot/firmware}
BEGIN_MARK='# clover begin'
END_MARK='# clover end'
WIFI_REGDOM=${CLOVER_WIFI_REGDOM:-GB}

# The block of config.txt; the model specific parts are in [pi4] and [pi5]
# (sources: NOTES.md, section "Образ (этап I)").
config_block() {
  cat << EOF
${BEGIN_MARK}
[all]
dtparam=i2c_arm=on
dtparam=spi=on
enable_uart=1
[pi4]
# GPIO14/15 is the PL011 UART (/dev/ttyAMA0) once Bluetooth is disabled
dtoverlay=disable-bt
[pi5]
# UART0 of RP1 on GPIO14/15 (/dev/ttyAMA0); /dev/serial0 is the debug connector (ttyAMA10)
dtoverlay=uart0-pi5
# Without a 5 A power supply the USB ports of Raspberry Pi 5 are limited to 600 mA in total.
# Uncomment only if the board is powered by a 5 V / 5 A source:
#usb_max_current_enable=1
[all]
${END_MARK}
EOF
}

# Reads config.txt from stdin, prints it with exactly one clover block at the end
config_apply() {
  sed "/^${BEGIN_MARK}\$/,/^${END_MARK}\$/d"
  config_block
}

# Reads cmdline.txt (one line) from stdin, prints one line: no serial console on the UART
# of the flight controller, regulatory domain for Wi-Fi
cmdline_apply() {
  local line token out=() regdom=0
  read -r line || true
  for token in $line; do
    case "$token" in
      console=serial0,* | console=ttyAMA0,* | console=ttyAMA10,* | console=ttyS0,*) continue;;
      cfg80211.ieee80211_regdom=*) regdom=1;;
      *) ;;
    esac
    out+=("$token")
  done
  if [[ $regdom == 0 ]]; then
    out+=("cfg80211.ieee80211_regdom=${WIFI_REGDOM}")
  fi
  echo "${out[*]}"
}

main() {
  require_chroot
  local file

  echo_stamp "#1 config.txt"
  file="${BOOT_DIR}/config.txt"
  [[ -f $file ]] || die "${file} not found: not an Ubuntu Raspberry Pi image?"
  config_apply < "$file" > "${file}.new"
  mv "${file}.new" "$file"
  if ! grep -q '^\[pi4\]$' "$file" || ! grep -q '^\[pi5\]$' "$file"; then
    die "[pi4]/[pi5] sections are missing in ${file}"
  fi

  echo_stamp "#2 cmdline.txt (no console on the serial port, Wi-Fi country ${WIFI_REGDOM})"
  file="${BOOT_DIR}/cmdline.txt"
  [[ -f $file ]] || die "${file} not found"
  cmdline_apply < "$file" > "${file}.new"
  mv "${file}.new" "$file"
  [[ $(wc -l < "$file") -eq 1 ]] || die "${file} must be a single line"

  echo_stamp "#3 No getty on the UART of the flight controller"
  systemctl mask serial-getty@ttyAMA0.service

  echo_stamp "#4 I2C, SPI modules and udev rules"
  printf 'i2c-dev\nspidev\n' > /etc/modules-load.d/clover.conf
  install -D -m 644 "${ASSETS_DIR}/99-clover-hw.rules" /etc/udev/rules.d/99-clover-hw.rules

  echo_stamp "#5 End of configure hardware interfaces" SUCCESS
}

# Sourcing (for tests) defines the functions only
if [[ ${BASH_SOURCE[0]} == "$0" ]]; then
  main "$@"
fi
