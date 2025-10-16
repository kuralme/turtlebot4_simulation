#!/bin/bash
# Basic entrypoint for ROS2/Docker containers

# Source ROS2 and Colcon workspaces
echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc
if [ -f /ros2_ws/install/setup.bash ]
then
  source /ros2_ws/install/setup.bash
  echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc
fi
echo "Sourced ROS2 workspace!"

# Set environment variables
export ROS_DOMAIN_ID=0
# export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(rospack find tb4_mpc)/models

# Adding Kill gz sim alias - effectively kill all gzserver and gzclient processes
echo "
kill_gzsim() {
    ps aux | grep gz | grep -v grep | awk '{print $2}' | xargs -r kill -9
}
" >> /root/.bashrc

# Execute the command passed into this entrypoint
exec "$@"