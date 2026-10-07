#!/usr/bin/env bash

#
# Script for image initialisation (runs inside the image chroot)
#
#   image-init.sh <IMAGE_VERSION> <SOURCE_IMAGE_URL>
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
USER_NAME=pi
USER_PASSWORD=raspberry
# Groups of the user; i2c, spi and gpio do not exist in the Ubuntu image and are created
USER_GROUPS=(sudo adm dialout video audio plugdev netdev input render i2c spi gpio)

main() {
  require_chroot
  local version=${1:?image version is required} source_url=${2:?source image URL is required}
  local group existing=()

  echo_stamp "Write Clover information"
  # Clover image version
  echo "$version" > /etc/clover_version
  # Origin image file name
  basename "${source_url%.*}" > /etc/clover_origin

  echo_stamp "Create the user ${USER_NAME}"
  for group in i2c spi gpio; do
    getent group "$group" > /dev/null || groupadd --system "$group"
  done
  if ! id "$USER_NAME" > /dev/null 2>&1; then
    useradd --create-home --shell /bin/bash --user-group "$USER_NAME"
  fi
  echo "${USER_NAME}:${USER_PASSWORD}" | chpasswd
  for group in "${USER_GROUPS[@]}"; do
    if getent group "$group" > /dev/null; then
      existing+=("$group")
    fi
  done
  usermod -aG "$(IFS=,; echo "${existing[*]}")" "$USER_NAME"
  # The home directory of Ubuntu is not world-traversable, nginx (www-data) needs it
  chmod o+x "/home/${USER_NAME}"
  # As on Raspberry Pi OS, where the Clover documentation comes from
  echo "${USER_NAME} ALL=(ALL) NOPASSWD: ALL" > "/etc/sudoers.d/010_${USER_NAME}-nopasswd"
  chmod 440 "/etc/sudoers.d/010_${USER_NAME}-nopasswd"

  echo_stamp "Seed cloud-init (user-data, network-config) and keep the hostname"
  install -m 644 "${ASSETS_DIR}/user-data" /boot/firmware/user-data
  install -m 644 "${ASSETS_DIR}/network-config" /boot/firmware/network-config
  mkdir -p /etc/cloud/cloud.cfg.d
  echo "preserve_hostname: true" > /etc/cloud/cloud.cfg.d/99-clover.cfg

  echo_stamp "Set max space for syslogs"
  # https://unix.stackexchange.com/questions/139513/how-to-clear-journalctl
  mkdir -p /etc/systemd/journald.conf.d
  printf '[Journal]\nSystemMaxUse=200M\n' > /etc/systemd/journald.conf.d/clover.conf

  echo_stamp "End of init image" SUCCESS
}

# Sourcing (for tests) defines the functions only
if [[ ${BASH_SOURCE[0]} == "$0" ]]; then
  main "$@"
fi
