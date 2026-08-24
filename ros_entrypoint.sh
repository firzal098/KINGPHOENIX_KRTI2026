#!/usr/bin/env bash
set -e

# Source ROS 2 base environment
if [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
    source "/opt/ros/${ROS_DISTRO}/setup.bash"
fi

# Source ROS 2 workspace overlay if built
if [ -f "/ros_ws/install/setup.bash" ]; then
    source "/ros_ws/install/setup.bash"
fi

exec "$@"
