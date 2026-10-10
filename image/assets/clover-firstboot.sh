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

# TEMPLATE: cmdline_regdom [CMDLINE_FILE]; Wi-Fi regulatory domain from the kernel command line (set by image-hardware.sh)
cmdline_regdom() {
  local token tokens=()
  read -ra tokens < "${1:-/proc/cmdline}" 2> /dev/null || true
  for token in "${tokens[@]}"; do
    case "$token" in
      cfg80211.ieee80211_regdom=*) echo "${token#*=}"; return 0;;
    esac
  done
  return 1
}

# A soft block of the radio (Ubuntu images keep it until a country is set) stops the access point
wifi_prepare() {
  local regdom
  rfkill unblock wifi || log "rfkill unblock failed"
  nmcli radio wifi on || log "nmcli radio wifi on failed"
  if regdom=$(cmdline_regdom /proc/cmdline); then
    iw reg set "$regdom" || log "iw reg set ${regdom} failed"
  else
    log "WARNING: no cfg80211.ieee80211_regdom in /proc/cmdline"
  fi
}

# TEMPLATE: wait_for_wifi_device <SECONDS>; the driver (brcmfmac) may load after this unit starts
wait_for_wifi_device() {
  local i
  for ((i = 0; i < $1; i++)); do
    if nmcli -t -f TYPE device | grep -qx wifi; then
      return 0
    fi
    sleep 1
  done
  return 1
}

# Everything an engineer needs to see in the journal when the access point does not come up
wifi_diagnostics() {
  {
    echo "--- rfkill list"; rfkill list
    echo "--- iw reg get"; iw reg get
    echo "--- nmcli device status"; nmcli device status
    echo "--- nmcli connection show"; nmcli connection show
    echo "--- iw list (supported interface modes)"; iw list | sed -n '/Supported interface modes/,/Band 1/p'
    echo "--- ip -br addr"; ip -br addr
    echo "--- ss -ulnp (ports 53, 67)"; ss -ulnp | grep -E ':(53|67)\b'
    echo "--- NetworkManager journal"; journalctl -u NetworkManager -b --no-pager -n 40
    echo "--- netplan files"; ls -l /etc/netplan /run/NetworkManager/system-connections
  } 2>&1 | sed 's/^/clover-firstboot: /' || true
}

# TEMPLATE: bring_up_ap <TRIES>
bring_up_ap() {
  local i
  for ((i = 1; i <= $1; i++)); do
    if nmcli connection up clover-ap; then
      return 0
    fi
    log "could not bring up clover-ap (attempt ${i} of $1)"
    sleep 3
  done
  return 1
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
    wifi_prepare
    nmcli connection reload || log "nmcli reload failed"
    if wait_for_wifi_device 30; then
      # NetworkManager may also start the profile itself (autoconnect); a second "up" is harmless
      bring_up_ap 3 || { log "WARNING: access point is down, NetworkManager keeps retrying"; wifi_diagnostics; }
    else
      log "WARNING: no Wi-Fi device in NetworkManager after 30 s (driver, firmware, unmanaged?)"
      wifi_diagnostics
    fi
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
