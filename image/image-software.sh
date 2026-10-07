#!/usr/bin/env bash

#
# Script for software installation (runs inside the image chroot)
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

export DEBIAN_FRONTEND=${DEBIAN_FRONTEND:-noninteractive}

# Not installed on purpose (see NOTES.md): pigpio, python3-pigpio, rpi_ws281x (no RP1 support),
# Butterfly, Node.js, gitbook, ptvsd, pyzbar, ntpdate, mjpg-streamer, the clever package.
PACKAGES=(
  # network
  network-manager
  wpasupplicant
  wireless-regdb
  iw
  rfkill
  avahi-daemon
  libnss-mdns
  openssh-server
  # web
  nginx
  # tools
  unzip
  zip
  rsync
  screen
  byobu
  tmux
  nmap
  lsof
  git
  tree
  vim
  tcpdump
  ipython3
  # hardware
  i2c-tools
  device-tree-compiler
  espeak-ng
  # build and python
  build-essential
  libffi-dev
  python3-dev
  python3-pip
  python3-flask
  python3-geopy
  python3-smbus2
  # geoid for mavros, geonav
  geographiclib-tools
  libgeographiclib-dev
)

main() {
  require_chroot

  echo_stamp "Increase apt retries"
  echo 'APT::Acquire::Retries "3";' > /etc/apt/apt.conf.d/80-retries

  echo_stamp "Update apt cache"
  retry apt-get update

  echo_stamp "Software installing"
  retry apt-get install -y --no-install-recommends "${PACKAGES[@]}"

  if [[ -f /usr/share/byobu/status/status ]]; then
    sed -i "s/updates_available//" /usr/share/byobu/status/status
  fi

  echo_stamp "Install pymavlink (used by selfcheck, no apt package in Ubuntu 24.04)"
  # PEP 668: --break-system-packages installs into /usr/local, the files of dpkg are not touched.
  # ROS 2 Python modules (rclpy, mavros) live in the system interpreter, so no virtualenv.
  retry pip3 install --no-cache-dir --break-system-packages "pymavlink==${CLOVER_PYMAVLINK_VERSION:-2.4.49}"

  echo_stamp "Add .vimrc"
  cat << EOF > /home/pi/.vimrc
set mouse-=a
syntax on
EOF
  chown pi:pi /home/pi/.vimrc

  echo_stamp "End of software installation" SUCCESS
}

# Sourcing (for tests) defines the functions only
if [[ ${BASH_SOURCE[0]} == "$0" ]]; then
  main "$@"
fi
