#!/usr/bin/env bash

#
# Script for network setup (runs inside the image chroot):
# NetworkManager, Wi-Fi access point clover-XXXX (192.168.11.1/24), avahi, first boot unit.
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
NM_CONNECTIONS_DIR=/etc/NetworkManager/system-connections

# Informational: what the first boot will find; the radio and the driver can be checked only on the board
show_network_state() {
  echo "--- AP profile (psk hidden)"
  sed 's/^psk=.*/psk=<hidden>/' "${NM_CONNECTIONS_DIR}/clover-ap.nmconnection"
  ls -l "${NM_CONNECTIONS_DIR}"
  echo "--- /etc/NetworkManager/conf.d, /usr/lib/NetworkManager/conf.d"
  grep -r . /etc/NetworkManager/conf.d /usr/lib/NetworkManager/conf.d 2>&1 || true
  echo "--- netplan and cloud-init network settings"
  ls -l /etc/netplan /boot/firmware/network-config 2>&1 || true
  grep -rs . /etc/netplan /etc/cloud/cloud.cfg.d 2>&1 | grep -i -E 'network|renderer|wifis|wlan|netplan' || true
  echo "--- dnsmasq: $(command -v dnsmasq || echo missing); dnsmasq.service: $(systemctl is-enabled dnsmasq.service 2>&1 || true)"
  echo "--- regulatory domain in cmdline.txt"
  grep -o 'cfg80211.ieee80211_regdom=[^ ]*' /boot/firmware/cmdline.txt 2>&1 || echo "(none)"
  echo "--- saved rfkill state"
  grep -r . /var/lib/systemd/rfkill 2>&1 || echo "(none)"
}

main() {
  require_chroot

  echo_stamp "#1 NetworkManager is the only network manager"
  # Not needed and slow down the boot (selfcheck measures it); network-online is not required by clover.service
  systemctl disable systemd-networkd-wait-online.service NetworkManager-wait-online.service || true
  systemctl enable NetworkManager.service
  install -D -m 644 "${ASSETS_DIR}/nm-clover.conf" /etc/NetworkManager/conf.d/90-clover.conf

  echo_stamp "#2 Wi-Fi access point profile clover-ap (SSID is set on the first boot)"
  install -d -m 755 "$NM_CONNECTIONS_DIR"
  install -m 600 "${ASSETS_DIR}/clover-ap.nmconnection" "${NM_CONNECTIONS_DIR}/clover-ap.nmconnection"

  echo_stamp "#3 Names clover and coex for the clients of the access point"
  install -D -m 644 "${ASSETS_DIR}/dnsmasq-clover.conf" /etc/NetworkManager/dnsmasq-shared.d/clover.conf

  echo_stamp "#3a The system dnsmasq must not run: the dnsmasq of NetworkManager needs ports 53/67 of the access point"
  # dnsmasq-base ships only the binary; the dnsmasq package would add a service listening on 0.0.0.0:53
  if [[ -e /lib/systemd/system/dnsmasq.service || -e /usr/lib/systemd/system/dnsmasq.service ]]; then
    systemctl disable dnsmasq.service || true
    systemctl mask dnsmasq.service
  fi
  command -v dnsmasq > /dev/null || die "dnsmasq-base is not installed: the access point (ipv4.method=shared) cannot start"

  echo_stamp "#4 avahi"
  systemctl enable avahi-daemon.service
  install -D -m 644 "${ASSETS_DIR}/sftp-ssh.service" /etc/avahi/services/sftp-ssh.service

  echo_stamp "#5 First boot unit (hostname clover-XXXX, SSID)"
  install -m 755 "${ASSETS_DIR}/clover-firstboot.sh" /usr/local/sbin/clover-firstboot
  install -m 644 "${ASSETS_DIR}/clover-firstboot.service" /etc/systemd/system/clover-firstboot.service
  systemctl enable clover-firstboot.service

  echo_stamp "#6 Network state of the image (for the build log)"
  show_network_state || true

  echo_stamp "#7 End of network installation" SUCCESS
}

# Sourcing (for tests) defines the functions only
if [[ ${BASH_SOURCE[0]} == "$0" ]]; then
  main "$@"
fi
