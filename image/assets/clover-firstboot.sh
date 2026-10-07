#!/usr/bin/env bash

#
# First boot setup of the Clover image: hostname and SSID clover-XXXX, hardware model.
# Replaces init_rpi.sh of the original image (which ran from /etc/rc.local).
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

STATE_DIR=/var/lib/clover
FLAG="${STATE_DIR}/firstboot.done"
AP_PROFILE=/etc/NetworkManager/system-connections/clover-ap.nmconnection

log() {
  echo "clover-firstboot: $*"
}

hardware_model() {
  # TEMPLATE: hardware_model <MODEL_STRING>; the string is /proc/device-tree/model
  case "$1" in
    *"Raspberry Pi 5"*) echo pi5;;
    *"Raspberry Pi 4"*) echo pi4;;
    *) echo unknown;;
  esac
}

# TEMPLATE: set_hosts_name <HOSTS_FILE> <HOSTNAME>
set_hosts_name() {
  if grep -q '^127\.0\.1\.1' "$1"; then
    sed -i "s/^127\\.0\\.1\\.1.*/127.0.1.1\\t${2} ${2}.local/" "$1"
  else
    printf '127.0.1.1\t%s %s.local\n' "$2" "$2" >> "$1"
  fi
}

# TEMPLATE: set_ap_ssid <KEYFILE> <SSID>
set_ap_ssid() {
  sed -i "s/^ssid=.*/ssid=${2}/" "$1"
  chmod 600 "$1"
}

random_suffix() {
  printf '%04d' $(($(od -An -N4 -tu4 /dev/urandom | tr -d ' ') % 10000))
}

main() {
  mkdir -p "$STATE_DIR"
  [[ ! -e $FLAG ]] || exit 0

  local model hw ssid
  model=$(tr -d '\0' < /proc/device-tree/model 2> /dev/null || true)
  hw=$(hardware_model "$model")
  echo "$hw" > /etc/clover_hw
  log "hardware: ${hw} (${model:-unknown model})"
  if [[ $hw == unknown ]]; then
    log "WARNING: only Raspberry Pi 4 and Raspberry Pi 5 are supported"
  fi

  ssid="clover-$(random_suffix)"
  log "SSID and hostname: ${ssid}"
  hostnamectl set-hostname "$ssid"
  set_hosts_name /etc/hosts "$ssid"

  if [[ -f $AP_PROFILE ]]; then
    set_ap_ssid "$AP_PROFILE" "$ssid"
    nmcli connection reload || log "nmcli reload failed"
    nmcli connection up clover-ap || log "could not bring up clover-ap (no wlan0?)"
  else
    log "WARNING: ${AP_PROFILE} not found"
  fi
  systemctl restart avahi-daemon.service || true

  touch "$FLAG"
}

# Sourcing (for tests) defines the functions only
if [[ ${BASH_SOURCE[0]} == "$0" ]]; then
  main "$@"
fi
