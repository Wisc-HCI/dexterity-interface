# Primitives

## Setup
1. This package depends on `robot_motion`, `robot_motion_interface`, and ROS 2 Jazzy. To install all of these dependencies build the docker container (instructions in the root readme) and run the following in the the container. 

2. Compile robot_motion_interface_ros and this package. Make sure you are in the `libs/robot_motion/ros` directory before running these. 
```bash
cd libs/robot_motion_interface/ros
colcon build --symlink-install

cd ../../../libs/primitives/ros
colcon build --symlink-install
cd ../../..
```


## Running
1. In one terminal launch either the real or simulated interface (or both)
```bash
source libs/robot_motion_interface/ros/install/setup.bash
# Launch Real
ros2 run robot_motion_interface_ros interface --ros-args -p interface_type:=bimanual -p config_path:=/workspace/libs/robot_motion_interface/config/bimanual_arm_config.yaml

# Launch simulation
ros2 run robot_motion_interface_ros interface --ros-args -p interface_type:=isaacsim -p config_path:=/workspace/libs/robot_motion_interface/config/isaacsim_config.yaml
```

2. In another terminal, launch the primitive node:
```bash
source libs/primitives/ros/install/setup.bash
ros2 run primitives_ros primitive_handler
```

Now, here are some topics you can test publishing to
```bash
source libs/primitives/ros/install/setup.bash
# Grasp with left gripper
ros2 topic pub --once /primitive/envelop_grasp std_msgs/msg/String "{data: 'left'}" 

# Grasp with right gripper
ros2 topic pub --once /primitive/envelop_grasp std_msgs/msg/String "{data: 'right'}" 

# Release with left gripper
ros2 topic pub --once /primitive/release std_msgs/msg/String "{data: 'left'}" 

# Release with right gripper
ros2 topic pub --once /primitive/release std_msgs/msg/String "{data: 'right'}" 

# Move left robot
ros2 topic pub /primitive/move_to_pose geometry_msgs/PoseStamped "{ header: {frame_id: 'left'}, pose: {position: {x: -0.2, y: 0.2, z: 0.4}, orientation: {x: 0.707, y: 0.707, z: 0.0, w: 0.0} }}" --once

# Move right robot
ros2 topic pub /primitive/move_to_pose geometry_msgs/PoseStamped "{ header: {frame_id: 'right'}, pose: {position: {x: 0.2, y: 0.2, z: 0.4}, orientation: {x: 0.707, y: 0.707, z: 0.0, w: 0.0} }}" --once
```


## Running Joy example
If you have a joystick controller (xbox controller), you can connect it via usb or bluetooth. Then you can teleop the robot with it.

Make sure you have joy ROS package installed (`sudo apt install ros-jazzy-joy`).

Then run both nodes in the prior section (#1, #2). After that launch these 2 nodes in seperate terminals:
```bash
ros2 run joy joy_node
source libs/primitives/ros/install/setup.bash
ros2 run primitives_ros joy_handler
```

TODO: Launch file for these, better home position