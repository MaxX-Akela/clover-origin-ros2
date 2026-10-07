#!/usr/bin/env bash

#
# Checks that the image scripts refuse to run outside of the image build.
# Safe to run on a developer machine: every case must exit with code 64 BEFORE anything
# is mounted or changed. The test itself never uses root, sudo, losetup or mount.
#
# Part of the ROS 2 port of Clover (https://github.com/CopterExpress/clover).
#
# Distributed under MIT License (available at https://opensource.org/licenses/MIT).
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#

set -euo pipefail

TEST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
IMAGE_DIR="$(cd "${TEST_DIR}/.." && pwd)"

if [[ $EUID -eq 0 ]]; then
  echo "Do not run this test as root" >&2
  exit 2
fi

failures=0

# TEMPLATE: expect_refused <DESCRIPTION> <ENV-ASSIGNMENTS-OR-env-ARGS...> -- <COMMAND...>
expect_refused() {
  local description=$1 rc=0
  shift
  env "$@" > /dev/null 2>&1 || rc=$?
  if [[ $rc -eq 64 ]]; then
    echo "ok: ${description}"
  else
    echo "FAILED: ${description} (exit code ${rc}, expected 64)"
    failures=$((failures + 1))
  fi
}

# Scripts that mount or attach loop devices
for script in scripts/image-chroot.sh scripts/image-resize.sh image-build.sh; do
  case "$script" in
    scripts/image-chroot.sh) args=(/nonexistent.img exec /bin/true);;
    scripts/image-resize.sh) args=(grow /nonexistent.img 8G);;
    *) args=();;
  esac
  expect_refused "${script}: no CI, no flag" -u CI bash "${IMAGE_DIR}/${script}" "${args[@]}"
  expect_refused "${script}: CI=false" CI=false bash "${IMAGE_DIR}/${script}" "${args[@]}"
  if grep -qi microsoft /proc/version 2> /dev/null; then
    expect_refused "${script}: CI=true is not enough under WSL" CI=true bash "${IMAGE_DIR}/${script}" "${args[@]}"
  fi
done

# Scripts that change the system they run in: CI=true and --i-know are not enough,
# the token of image-chroot.sh is required
for script in image-init.sh image-software.sh image-ros.sh image-network.sh image-hardware.sh image-validate.sh image-cleanup.sh; do
  expect_refused "${script}: outside the chroot" -u CI bash "${IMAGE_DIR}/${script}" 1 http://example.invalid/x.img.xz
  expect_refused "${script}: CI=true outside the chroot" CI=true bash "${IMAGE_DIR}/${script}" 1 http://example.invalid/x.img.xz
  expect_refused "${script}: --i-know outside the chroot" CI=true bash "${IMAGE_DIR}/${script}" --i-know
  expect_refused "${script}: fake token outside the chroot" CLOVER_CHROOT_TOKEN=abc bash "${IMAGE_DIR}/${script}" 1 http://example.invalid/x.img.xz
done

# Changes apt sources: CI or chroot only
expect_refused "scripts/install-ros-apt-source.sh: no CI" -u CI bash "${IMAGE_DIR}/scripts/install-ros-apt-source.sh"

if ((failures > 0)); then
  echo "${failures} case(s) failed"
  exit 1
fi
echo "All guard cases passed"
