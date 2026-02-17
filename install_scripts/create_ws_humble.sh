#!/usr/bin/env bash

# Create ROS2 Humble workspace

WS_PATH=~/ros2_ws

source /opt/ros/humble/setup.bash
mkdir -p $WS_PATH/src
cd $WS_PATH
colcon build --symlink-install

echo ' ' >> ~/.bashrc
echo '# ROS2 workspace setup' >> ~/.bashrc
echo "source $WS_PATH/install/setup.bash" >> ~/.bashrc
source ~/.bashrc

echo "ROS2 workspace created at $WS_PATH"
