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

# Auto install GCS dependencies if node_modules is missing
if [ -f "/ros_ws/src/gcs/package.json" ] && [ ! -d "/ros_ws/src/gcs/node_modules" ]; then
    echo "[Entrypoint] Initializing GCS dependencies (npm install)..."
    (cd /ros_ws/src/gcs && npm install --silent)
fi

# Strip any leading '--' occurrences
while [ "$1" = "--" ]; do
    shift
done

# If arguments remain, execute them; otherwise start bash
if [ $# -gt 0 ]; then
    exec "$@"
else
    exec bash
fi