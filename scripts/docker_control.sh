#!/bin/bash
# Run inside the ros-base container.
# Builds ROS packages
set -e

export ROS_STATIC_PEERS=192.168.4.9
export ROS_AUTOMATIC_DISCORY_RANGE=SUBNET

cd /workspace/libs/robot_motion_interface/ros
colcon build --cmake-clean-cache --symlink-install

cd /workspace/libs/primitives/ros
colcon build --cmake-clean-cache --symlink-install

cd /workspace
source libs/robot_motion_interface/ros/install/setup.bash
source libs/primitives/ros/install/setup.bash

exec bash

