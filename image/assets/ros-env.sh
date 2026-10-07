# ROS 2 environment of the Clover image. Sourced by clover.service and ~/.bashrc.
# shellcheck shell=bash
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
if [ -f /opt/mavros_ws/install/setup.bash ]; then
  source /opt/mavros_ws/install/setup.bash
fi
if [ -f /home/pi/ros2_ws/install/setup.bash ]; then
  source /home/pi/ros2_ws/install/setup.bash
fi
# Discovery only inside the subnet of the access point (ROS_HOSTNAME / ROS_IP do not exist in ROS 2)
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
