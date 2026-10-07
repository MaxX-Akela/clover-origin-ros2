#!/usr/bin/env bash

#
# Configure the ROS 2 apt repository (the ros2-apt-source package of the ROS documentation).
# Used by the CI preflight job on the runner and by image-ros.sh inside the image chroot.
# The version is pinned, so the unauthenticated GitHub API is not needed.
#
# Part of the ROS 2 port of Clover (https://github.com/CopterExpress/clover).
#
# Distributed under MIT License (available at https://opensource.org/licenses/MIT).
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=common.sh
source "${SCRIPT_DIR}/common.sh"

ROS_APT_SOURCE_VERSION=${ROS_APT_SOURCE_VERSION:-1.3.0}

main() {
  guard_init "$@"
  require_ci_or_chroot

  local codename deb url
  codename=$(. /etc/os-release && echo "${UBUNTU_CODENAME:-${VERSION_CODENAME}}")
  [[ $codename == noble ]] || die "Ubuntu 24.04 (noble) is required, this is '${codename}'"
  if dpkg -s ros2-apt-source > /dev/null 2>&1; then
    echo_stamp "ros2-apt-source is already installed"
  else
    deb=$(mktemp --suffix=.deb)
    url="https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.${codename}_all.deb"
    retry apt-get update
    retry apt-get install -y --no-install-recommends curl ca-certificates
    retry curl -fsSL -o "$deb" "$url"
    dpkg -i "$deb"
    rm -f "$deb"
  fi
  retry apt-get update
}

# Sourcing (for tests) defines the functions only
if [[ ${BASH_SOURCE[0]} == "$0" ]]; then
  main "$@"
fi
