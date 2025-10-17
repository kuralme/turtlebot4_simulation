#!/bin/bash

# Exit on errors
set -e

# Source ROS2
source /opt/ros/${ROS_DISTRO}/setup.bash

# Source workspace if built
if [ -f /ros2_ws/install/setup.bash ]; then
  source /ros2_ws/install/setup.bash
fi

# Setup ROS_DOMAIN_ID
export ROS_DOMAIN_ID=0

# Execute the provided command
exec bash -c "$@"
