#!/bin/bash
# Run inside the isaac-base container (Terminal 1).
# Builds ROS packages.

export ROS_STATIC_PEERS=192.168.4.4
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET

cd /workspace/libs/robot-stack/robot_motion_interface/ros
colcon build --cmake-clean-cache --symlink-install

cd /workspace/libs/primitives/ros
colcon build --cmake-clean-cache --symlink-install

cd /workspace
source libs/robot-stack/robot_motion_interface/ros/install/setup.bash
source libs/primitives/ros/install/setup.bash


