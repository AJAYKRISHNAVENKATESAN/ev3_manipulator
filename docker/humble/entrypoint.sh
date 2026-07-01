#!/bin/bash
set -e
source /opt/ros/${ROS_DISTRO}/setup.bash
if [ -f /workspace/install/setup.bash ]; then
    source /workspace/install/setup.bash
fi
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
echo "=================================================="
echo " ev3_manipulator dev  |  ROS 2 ${ROS_DISTRO} + Gazebo Fortress (Ignition)"
echo " aliases: cb (build) · cs (source) · ws (workspace)"
echo "=================================================="
exec "$@"
