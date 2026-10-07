#!/usr/bin/env bash

#
# Script for building the Clover image for Raspberry Pi 4 and Raspberry Pi 5
# (Ubuntu 24.04 arm64, ROS 2 Jazzy). Native arm64 host, no Docker, no qemu.
# It refuses to run outside of CI unless --i-know is given.
#
#   sudo CI=true image/image-build.sh
#
# Environment (all optional):
#   CLOVER_IMAGE_VERSION  version in the file name and /etc/clover_version (default: git describe)
#   SOURCE_IMAGE_URL, SOURCE_IMAGE_SHA256   base image and its checksum (checked always)
#   IMAGE_SIZE            size of the working image, default 8G
#   IMAGES_DIR, CACHE_DIR output and download cache, default image/out/images and image/out/cache
#   CLOVER_MAVROS_SOURCE  auto (default) | apt | source
#   CLOVER_BUILD_JOBS     compiler jobs inside the image
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

REPO_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

# https://cdimage.ubuntu.com/releases/24.04/release/ (SHA256SUMS in the same directory)
SOURCE_IMAGE_URL=${SOURCE_IMAGE_URL:-"https://cdimage.ubuntu.com/releases/24.04/release/ubuntu-24.04.5-preinstalled-server-arm64+raspi.img.xz"}
SOURCE_IMAGE_SHA256=${SOURCE_IMAGE_SHA256:-"b23371a5c8d02f612c26e7c18cacb81c5a48f968c7c66a2645391320aa94351e"}
IMAGE_SIZE=${IMAGE_SIZE:-8G}
IMAGES_DIR=${IMAGES_DIR:-"${SCRIPT_DIR}/out/images"}
CACHE_DIR=${CACHE_DIR:-"${SCRIPT_DIR}/out/cache"}

GUARD_FLAG=()

chroot_exec() {
  "${SCRIPT_DIR}/scripts/image-chroot.sh" "${GUARD_FLAG[@]}" "$IMAGE_PATH" exec "$@"
}

chroot_copy() {
  "${SCRIPT_DIR}/scripts/image-chroot.sh" "${GUARD_FLAG[@]}" "$IMAGE_PATH" copy "$@"
}

# Runs on any exit. Every child script cleans up after itself; this is the last resort:
# print the state on failure and detach loop devices that still point to the working image.
cleanup() {
  local rc=$? dev
  set +e
  if [[ $rc -ne 0 ]]; then
    echo_stamp "Build failed with code ${rc}, state of loop devices and mounts:" ERROR >&2
    losetup -a >&2
    mount | grep clover >&2
    lsblk -f >&2
  fi
  if [[ -n ${IMAGE_PATH:-} && -f $IMAGE_PATH ]]; then
    while IFS=: read -r dev _; do
      [[ -n $dev ]] || continue
      echo_stamp "Detaching leftover ${dev}" ERROR >&2
      grep " $(realpath "${dev}")p" /proc/mounts | awk '{print $2}' | sort -r | xargs -r -n1 umount -l
      losetup -d "$dev"
    done < <(losetup -j "$IMAGE_PATH")
  fi
  exit "$rc"
}

image_version() {
  if [[ -n ${CLOVER_IMAGE_VERSION:-} ]]; then
    echo "$CLOVER_IMAGE_VERSION"
  else
    git -C "$REPO_DIR" describe --tags --always 2> /dev/null || echo dev
  fi
}

get_image() {
  # TEMPLATE: get_image <XZ_PATH>; downloads if needed and always verifies the checksum
  local xz_path=$1
  if [[ ! -f $xz_path ]] || ! echo "${SOURCE_IMAGE_SHA256}  ${xz_path}" | sha256sum -c --status -; then
    echo_stamp "Downloading original Linux distribution"
    retry curl -fL --retry 3 -o "${xz_path}.part" "$SOURCE_IMAGE_URL"
    mv "${xz_path}.part" "$xz_path"
  else
    echo_stamp "Linux distribution already downloaded"
  fi
  echo "${SOURCE_IMAGE_SHA256}  ${xz_path}" | sha256sum -c - || die "Checksum of ${xz_path} does not match SOURCE_IMAGE_SHA256"
}

main() {
  guard_init "$@"
  require_ci
  if [[ $I_KNOW == 1 ]]; then
    GUARD_FLAG=(--i-know)
  fi
  [[ $(uname -m) == aarch64 ]] || die "Native arm64 host is required, this is $(uname -m)"

  trap cleanup EXIT
  trap 'exit 130' INT
  trap 'exit 143' TERM

  local version tool
  for tool in curl xz sha256sum rsync sfdisk losetup unshare truncate numfmt; do
    command -v "$tool" > /dev/null || die "${tool} is required"
  done

  version=$(image_version)
  IMAGE_PATH="${IMAGES_DIR}/clover-ros2_${version//\//-}.img"
  mkdir -p "$IMAGES_DIR" "$CACHE_DIR"
  echo_stamp "Building ${IMAGE_PATH} (version ${version})"
  df -h "$IMAGES_DIR" || true

  get_image "${CACHE_DIR}/$(basename "$SOURCE_IMAGE_URL")"
  echo_stamp "Unpacking the image"
  xz -dc "${CACHE_DIR}/$(basename "$SOURCE_IMAGE_URL")" > "$IMAGE_PATH"

  # Make free space
  "${SCRIPT_DIR}/scripts/image-resize.sh" "${GUARD_FLAG[@]}" grow "$IMAGE_PATH" "$IMAGE_SIZE"

  # Build scripts and assets go to the image, the repository comes after the user pi exists
  chroot_copy "${SCRIPT_DIR}" /root/clover-image --exclude=/out
  chroot_exec /root/clover-image/image-init.sh "$version" "$SOURCE_IMAGE_URL"

  # Copy the repository to the image, without git history and build artifacts
  chroot_copy "$REPO_DIR" /home/pi/ros2_ws/src/clover-origin-ros2 \
    --exclude=/.git --exclude=/build --exclude=/install --exclude=/log \
    --exclude=/image/out --exclude=__pycache__

  chroot_exec /root/clover-image/image-software.sh
  chroot_exec /root/clover-image/image-ros.sh "$version"
  chroot_exec /root/clover-image/image-network.sh
  chroot_exec /root/clover-image/image-hardware.sh
  chroot_exec /root/clover-image/image-validate.sh
  chroot_exec /root/clover-image/image-cleanup.sh

  "${SCRIPT_DIR}/scripts/image-resize.sh" "${GUARD_FLAG[@]}" shrink "$IMAGE_PATH"

  echo_stamp "Compressing the image"
  xz -T0 -6 --force "$IMAGE_PATH"
  (cd "$IMAGES_DIR" && sha256sum "$(basename "${IMAGE_PATH}.xz")" > "$(basename "${IMAGE_PATH}.xz").sha256")
  ls -l "$IMAGES_DIR"
  echo_stamp "Image is ready: ${IMAGE_PATH}.xz (not tested on hardware)" SUCCESS
}

# Sourcing (for tests) defines the functions only
if [[ ${BASH_SOURCE[0]} == "$0" ]]; then
  main "$@"
fi
