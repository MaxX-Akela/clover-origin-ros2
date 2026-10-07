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

  echo_stamp "#4 avahi"
  systemctl enable avahi-daemon.service
  install -D -m 644 "${ASSETS_DIR}/sftp-ssh.service" /etc/avahi/services/sftp-ssh.service

  echo_stamp "#5 First boot unit (hostname clover-XXXX, SSID)"
  install -m 755 "${ASSETS_DIR}/clover-firstboot.sh" /usr/local/sbin/clover-firstboot
  install -m 644 "${ASSETS_DIR}/clover-firstboot.service" /etc/systemd/system/clover-firstboot.service
  systemctl enable clover-firstboot.service

  echo_stamp "#6 End of network installation" SUCCESS
}

# Sourcing (for tests) defines the functions only
if [[ ${BASH_SOURCE[0]} == "$0" ]]; then
  main "$@"
fi
