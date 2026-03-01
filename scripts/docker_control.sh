#!/bin/bash
# Run inside the ros-base container.
# Builds ROS packages
set -e

cd /workspace/libs/robot_motion_interface/ros
colcon build --cmake-clean-cache --symlink-install

cd /workspace/libs/primitives/ros
colcon build --cmake-clean-cache --symlink-install

cd /workspace
source libs/robot_motion_interface/ros/install/setup.bash
source libs/primitives/ros/install/setup.bash

exec bash

