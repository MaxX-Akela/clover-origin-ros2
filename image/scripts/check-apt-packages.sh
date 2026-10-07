#!/usr/bin/env bash

#
# Preflight check of the apt packages the image needs, before the long image build.
# Reads only the apt cache (apt-cache policy), mounts nothing, changes nothing.
# Run it after the ROS 2 apt source is configured and `apt-get update` is done.
#
# The result is a table in $GITHUB_STEP_SUMMARY (if set) and the variable
#   mavros_source=apt|source
# in $GITHUB_OUTPUT (if set) and on stdout. mavros comes from apt if
# ros-jazzy-mavros and its friends have version >= MAVROS_MIN_VERSION.
# Exit code 1: a required package has no installation candidate.
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

MAVROS_MIN_VERSION=${MAVROS_MIN_VERSION:-2.16.0}

# mavros family: a missing or old package switches the build to the source fallback
MAVROS_PACKAGES=(
  ros-jazzy-mavros
  ros-jazzy-mavros-extras
  ros-jazzy-mavros-msgs
  ros-jazzy-libmavconn
)

# Required: the image build fails without them
ROS_PACKAGES=(
  ros-jazzy-ros-base
  ros-jazzy-angles
  ros-jazzy-camera-ros
  ros-jazzy-v4l2-camera
  ros-jazzy-image-proc
  ros-jazzy-topic-tools
  ros-jazzy-web-video-server
  ros-jazzy-rosbridge-server
  ros-jazzy-tf2-web-republisher
  ros-jazzy-image-geometry
  ros-jazzy-rmw-cyclonedds-cpp
)

UBUNTU_PACKAGES=(
  network-manager
  wireless-regdb
  avahi-daemon
  libnss-mdns
  nginx
  openssh-server
  i2c-tools
  device-tree-compiler
  espeak-ng
  python3-pip
  python3-flask
  python3-geopy
  python3-smbus2
  geographiclib-tools
  libgeographiclib-dev
)

# Expected to be absent in noble (see NOTES.md); only reported
INFO_PACKAGES=(
  libraspberrypi-bin
  python3-pymavlink
)

candidate() {
  # empty output if there is no package or no candidate
  local value
  value=$(apt-cache policy "$1" 2> /dev/null | awk '/Candidate:/ {print $2}')
  if [[ $value == '(none)' ]]; then
    value=
  fi
  echo "$value"
}

failures=0
mavros_ok=1
rows=()

check_group() {
  # TEMPLATE: check_group <GROUP> <REQUIRED:yes|no|mavros> <PACKAGES...>
  local group=$1 mode=$2 pkg ver status
  shift 2
  for pkg in "$@"; do
    ver=$(candidate "$pkg")
    status=ok
    if [[ -z $ver ]]; then
      case "$mode" in
        yes) status=MISSING; failures=$((failures + 1));;
        mavros) status=MISSING; mavros_ok=0;;
        *) status='absent (expected)';;
      esac
    elif [[ $mode == mavros ]] && ! version_ge "$ver" "$MAVROS_MIN_VERSION"; then
      status="older than ${MAVROS_MIN_VERSION}"
      mavros_ok=0
    fi
    rows+=("| ${group} | \`${pkg}\` | ${ver:--} | ${status} |")
  done
}

main() {
  local arch source summary
  arch=$(dpkg --print-architecture)

  check_group mavros mavros "${MAVROS_PACKAGES[@]}"
  check_group ros yes "${ROS_PACKAGES[@]}"
  check_group ubuntu yes "${UBUNTU_PACKAGES[@]}"
  check_group info no "${INFO_PACKAGES[@]}"

  if [[ $mavros_ok == 1 ]]; then
    source=apt
  else
    source=source
  fi

  summary="### apt packages for the image (${arch})

| Group | Package | Candidate | Status |
|---|---|---|---|
$(printf '%s\n' "${rows[@]}")

mavros_source=${source} (minimum ${MAVROS_MIN_VERSION})"
  echo "$summary"
  if [[ -n ${GITHUB_STEP_SUMMARY:-} ]]; then
    echo "$summary" >> "$GITHUB_STEP_SUMMARY"
  fi
  echo "mavros_source=${source}"
  if [[ -n ${GITHUB_OUTPUT:-} ]]; then
    echo "mavros_source=${source}" >> "$GITHUB_OUTPUT"
  fi

  # The image needs noble-updates: without it the -dev libraries of ROS do not match the installed libraries
  echo "--- candidates of the libraries that failed in the image build"
  apt-cache policy liblz4-1 liblz4-dev libzstd1 libzstd-dev | grep -E '^[a-z]|Candidate|Installed' || true
  if ! apt-cache policy | grep -q 'a=noble-updates'; then
    echo_stamp "noble-updates is not among the apt sources of this system" ERROR >&2
    failures=$((failures + 1))
  fi
  echo "--- simulation: apt-get -s install ros-jazzy-ros-base ros-dev-tools python3-rosdep (informational, state of the runner)"
  apt-get -s install ros-jazzy-ros-base ros-dev-tools python3-rosdep | tail -n 15 || echo_stamp "the simulation failed on this system" ERROR >&2

  if [[ $arch != arm64 ]]; then
    echo_stamp "Architecture is ${arch}, not arm64: the candidates above are not the arm64 ones" ERROR >&2
  fi
  if ((failures > 0)); then
    die "${failures} required package(s) have no installation candidate"
  fi
  if [[ $source == source ]]; then
    echo_stamp "mavros >= ${MAVROS_MIN_VERSION} is not in apt, the image build will use the source fallback"
  fi
}

# Sourcing (for tests) defines the functions only
if [[ ${BASH_SOURCE[0]} == "$0" ]]; then
  main "$@"
fi
