#!/usr/bin/env bash

#
# Script for ROS 2 Jazzy installation and the Clover build (runs inside the image chroot)
#
#   image-ros.sh <IMAGE_VERSION>
#
# Environment (forwarded by image-chroot.sh):
#   CLOVER_MAVROS_SOURCE  auto (default) | apt | source
#   CLOVER_BUILD_JOBS     compiler jobs, default: half of the cores (at least 1)
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
ROS_DISTRO=jazzy
USER_NAME=pi
USER_HOME="/home/${USER_NAME}"
WS_DIR="${USER_HOME}/ros2_ws"
MAVROS_WS_DIR=/opt/mavros_ws
MAVROS_VERSION=2.16.0
export DEBIAN_FRONTEND=${DEBIAN_FRONTEND:-noninteractive}

# Packages that are not in package.xml of our packages but are needed by the image
EXTRA_ROS_PACKAGES=(
  "ros-${ROS_DISTRO}-v4l2-camera"
  "ros-${ROS_DISTRO}-image-proc"
  "ros-${ROS_DISTRO}-topic-tools"
  "ros-${ROS_DISTRO}-web-video-server"
  "ros-${ROS_DISTRO}-rosbridge-server"
  "ros-${ROS_DISTRO}-tf2-web-republisher"
  "ros-${ROS_DISTRO}-image-geometry"
  "ros-${ROS_DISTRO}-camera-ros"
  "ros-${ROS_DISTRO}-rmw-cyclonedds-cpp"
  "ros-${ROS_DISTRO}-angles"
)

as_user() {
  runuser -u "$USER_NAME" -- env "HOME=${USER_HOME}" "USER=${USER_NAME}" "LOGNAME=${USER_NAME}" "$@"
}

build_jobs() {
  local jobs=${CLOVER_BUILD_JOBS:-}
  if [[ -z $jobs ]]; then
    jobs=$(($(nproc) / 2))
  fi
  if ((jobs < 1)); then
    jobs=1
  fi
  echo "$jobs"
}

# Prints apt or source. auto: apt if the candidate of ros-jazzy-mavros is >= MAVROS_VERSION.
mavros_source() {
  local requested=${CLOVER_MAVROS_SOURCE:-auto} candidate
  case "$requested" in
    apt | source) echo "$requested"; return;;
    auto) ;;
    *) die "CLOVER_MAVROS_SOURCE must be auto, apt or source, not '${requested}'";;
  esac
  candidate=$(apt-cache policy "ros-${ROS_DISTRO}-mavros" | awk '/Candidate:/ {print $2}')
  if [[ -n $candidate && $candidate != '(none)' ]] && version_ge "$candidate" "$MAVROS_VERSION"; then
    echo apt
  else
    echo source
  fi
}

install_mavros_from_source() {
  echo_stamp "Build mavros ${MAVROS_VERSION} from source in ${MAVROS_WS_DIR}"
  mkdir -p "${MAVROS_WS_DIR}/src"
  chown -R "${USER_NAME}:${USER_NAME}" "$MAVROS_WS_DIR"
  retry as_user git clone --depth 1 --branch "$MAVROS_VERSION" https://github.com/mavlink/mavros.git "${MAVROS_WS_DIR}/src/mavros"
  retry as_user bash -c "cd ${MAVROS_WS_DIR} && . /opt/ros/${ROS_DISTRO}/setup.bash && \
    rosdep install -y --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} --skip-keys='ament_lint_auto ament_lint_common ament_cmake_google_benchmark' -r"
  retry apt-get install -y --no-install-recommends "ros-${ROS_DISTRO}-angles" "ros-${ROS_DISTRO}-mavlink"
  as_user bash -c "cd ${MAVROS_WS_DIR} && . /opt/ros/${ROS_DISTRO}/setup.bash && \
    MAKEFLAGS=-j$(build_jobs) colcon build --packages-select libmavconn mavros_msgs mavros mavros_extras \
      --parallel-workers 1 --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release"
  [[ -f ${MAVROS_WS_DIR}/install/setup.bash ]] || die "mavros build did not produce ${MAVROS_WS_DIR}/install/setup.bash"
}

install_geographiclib_datasets() {
  # mavros_node does not start without the geoid (egm96-5), the system directory is used
  echo_stamp "Install GeographicLib datasets (needed for mavros)"
  retry "/opt/ros/${ROS_DISTRO}/lib/mavros/install_geographiclib_datasets.sh"
  [[ -f /usr/share/GeographicLib/geoids/egm96-5.pgm ]] || die "egm96-5.pgm was not installed"
}

build_clover() {
  local jobs
  jobs=$(build_jobs)
  echo_stamp "Build Clover with ${jobs} job(s)"
  as_user bash -c "
    . /etc/clover/ros-env.sh
    cd ${WS_DIR}
    export MAKEFLAGS=-j${jobs} CMAKE_BUILD_PARALLEL_LEVEL=${jobs}
    colcon build --packages-select aruco_pose led_msgs roswww_static clover \
      --parallel-workers 1 --symlink-install \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
  "
  [[ -f ${WS_DIR}/install/setup.bash ]] || die "colcon build did not produce ${WS_DIR}/install/setup.bash"
}

main() {
  require_chroot
  local version=${1:?image version is required} mavros_from
  echo_stamp "Clover image version ${version}"

  echo_stamp "Install the ROS 2 apt repository"
  "${SCRIPT_DIR}/scripts/install-ros-apt-source.sh"

  echo_stamp "apt state before the installation of ROS 2"
  apt_diag liblz4-1 liblz4-dev libzstd1 libzstd-dev "ros-${ROS_DISTRO}-ros-base"
  apt_upgrade

  echo_stamp "Install ROS 2 ${ROS_DISTRO}"
  retry apt-get install -y --no-install-recommends "ros-${ROS_DISTRO}-ros-base" ros-dev-tools python3-rosdep     || { apt_diag liblz4-1 liblz4-dev libzstd1 libzstd-dev; die "Installation of ROS 2 ${ROS_DISTRO} failed"; }

  echo_stamp "Environment of ROS 2"
  mkdir -p /etc/clover
  install -m 644 "${ASSETS_DIR}/ros-env.sh" /etc/clover/ros-env.sh

  echo_stamp "Init rosdep"
  if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
    retry rosdep init
  fi
  retry rosdep update --rosdistro "$ROS_DISTRO"
  retry as_user rosdep update --rosdistro "$ROS_DISTRO"

  echo_stamp "Check that the workspace is in place"
  [[ -d ${WS_DIR}/src ]] || die "${WS_DIR}/src does not exist (the repository was not copied)"
  chown -R "${USER_NAME}:${USER_NAME}" "$WS_DIR"

  mavros_from=$(mavros_source)
  echo_stamp "mavros: ${mavros_from}"
  if [[ $mavros_from == source ]]; then
    install_mavros_from_source
  else
    retry apt-get install -y --no-install-recommends "ros-${ROS_DISTRO}-mavros" "ros-${ROS_DISTRO}-mavros-extras"
  fi

  echo_stamp "Install dependencies of the Clover packages"
  retry bash -c ". /opt/ros/${ROS_DISTRO}/setup.bash && cd ${WS_DIR} && \
    rosdep install -y --from-paths src --ignore-src --rosdistro ${ROS_DISTRO}"
  retry apt-get install -y --no-install-recommends "${EXTRA_ROS_PACKAGES[@]}"

  install_geographiclib_datasets
  build_clover

  echo_stamp "Update www (symlinks in ~/.ros/www for nginx)"
  as_user bash -c ". /etc/clover/ros-env.sh && ros2 run roswww_static update"

  echo_stamp "Setup nginx (replaces monkey): ~/.ros/www on port 80"
  install -D -m 644 "${ASSETS_DIR}/nginx-clover.conf" /etc/nginx/sites-available/clover
  rm -f /etc/nginx/sites-enabled/default
  ln -sf /etc/nginx/sites-available/clover /etc/nginx/sites-enabled/clover
  nginx -t
  systemctl enable nginx.service

  echo_stamp "Make \$HOME/examples symlink"
  ln -sfn "${WS_DIR}/install/clover/share/clover/examples" "${USER_HOME}/examples"

  echo_stamp "Make udev rules symlinks"
  local rule
  for rule in "${WS_DIR}"/install/clover/share/clover/udev/*.rules; do
    ln -sf "$rule" "/etc/udev/rules.d/$(basename "$rule")"
  done

  echo_stamp "Install and enable clover.service"
  install -m 644 "${ASSETS_DIR}/clover.service" /etc/systemd/system/clover.service
  systemctl enable clover.service

  echo_stamp "Setup ROS environment"
  sed -i '/^# clover begin$/,/^# clover end$/d' "${USER_HOME}/.bashrc"
  cat << 'EOF' >> "${USER_HOME}/.bashrc"
# clover begin
export LANG=C.UTF-8
export LC_ALL=C.UTF-8
source /etc/clover/ros-env.sh
if command -v register-python-argcomplete3 > /dev/null; then
  eval "$(register-python-argcomplete3 ros2)"
  eval "$(register-python-argcomplete3 colcon)"
fi
# clover end
EOF

  chown -R "${USER_NAME}:${USER_NAME}" "$USER_HOME"
  echo_stamp "Clean apt cache"
  apt-get clean -qq > /dev/null
  echo_stamp "END of ROS INSTALLATION" SUCCESS
}

# Sourcing (for tests) defines the functions only
if [[ ${BASH_SOURCE[0]} == "$0" ]]; then
  main "$@"
fi
