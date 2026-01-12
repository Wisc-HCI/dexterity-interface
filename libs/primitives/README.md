# Primitives

## Setup
1. This package depends on `robot_motion`, `robot_motion_interface`, and ROS 2 Jazzy. To install all of these dependencies build the docker container (instructions in the root readme) and run the following in the the container. 

2. Compile robot_motion_interface_ros and this package. Make sure you are in the `libs/robot_motion/ros` directory before running these. 
```bash
cd libs/robot_motion_interface/ros
colcon build --symlink-install
cd ../../..
cd libs/primitives/ros
colcon build --symlink-install
cd ../../..
```


## Running Action Handler
1. In one terminal launch either the real or simulated files:
```bash
source libs/robot_motion_interface/ros/install/setup.bash
source libs/primitives/ros/install/setup.bash
# OPTION 1: Launch Real
ros2 launch primitives sim.launch.py

# OPTION 2: Launch simulation
ros2 launch primitives real.launch.py
```

TODO: DOC of higher level prims
Now, here are some actions you can test 
```bash
source libs/primitives/ros/install/setup.bash

# Home both robots
ros2 action send_goal /primitives primitive_msgs_ros/action/Primitives "
primitives:
- type: home"

# Move right robot
ros2 action send_goal /primitives primitive_msgs_ros/action/Primitives "
primitives:
- type: move_to_pose
  arm: right
  pose:
    pose:
      position: {x: 0.2, y: 0.2, z: 0.3}
      orientation: { x: 0.707, y: 0.707, z: 0.0, w: 0.0 }"


# Move left robot
ros2 action send_goal /primitives primitive_msgs_ros/action/Primitives "
primitives:
- type: move_to_pose
  arm: left
  pose:
    pose:
      position: {x: -0.2, y: 0.2, z: 0.2}
      orientation: { x: 0.707, y: 0.707, z: 0.0, w: 0.0 }"

# Envelop grasp with left robot
ros2 action send_goal /primitives primitive_msgs_ros/action/Primitives "
primitives:
- type: envelop_grasp
  arm: left"

# Release grasp with left robot
ros2 action send_goal /primitives primitive_msgs_ros/action/Primitives "
primitives:
- type: release
  arm: left"

# Chain of Primitives:
ros2 action send_goal /primitives primitive_msgs_ros/action/Primitives "
primitives:
- type: home
- type: envelop_grasp
  arm: left
- type: move_to_pose
  arm: left
  pose:
    pose:
      position: {x: -0.2, y: 0.2, z: 0.2}
      orientation: { x: 0.707, y: 0.707, z: 0.0, w: 0.0 }
- type: release
  arm: left
- type: envelop_grasp
  arm: right
- type: move_to_pose
  arm: right
  pose:
    pose:
      position: {x: 0.2, y: 0.2, z: 0.2}
      orientation: { x: 0.707, y: 0.707, z: 0.0, w: 0.0 }
- type: release
  arm: right
"
```


## Running Topic Handler
TODO: EVENTUALLY REMOVE THISf
1. In one terminal launch either the real or simulated interface (or both)
```bash
source libs/robot_motion_interface/ros/install/setup.bash
# OPTION 1: Launch Real
ros2 run robot_motion_interface_ros interface --ros-args -p interface_type:=bimanual -p config_path:=/workspace/libs/robot_motion_interface/config/bimanual_arm_config.yaml

# OPTION 2: Launch simulation
ros2 run robot_motion_interface_ros interface --ros-args -p interface_type:=isaacsim -p config_path:=/workspace/libs/robot_motion_interface/config/isaacsim_config.yaml
```

2. In another terminal, launch the primitive node:
```bash
source libs/primitives/ros/install/setup.bash
ros2 run primitives_ros primitive_topic_handler
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

Make sure you have joy ROS package installed: `sudo apt install ros-jazzy-joy`.


1. Then in one terminal launch either the real or simulated interface (or both)
```bash
source libs/robot_motion_interface/ros/install/setup.bash
# Launch Real
ros2 run robot_motion_interface_ros interface --ros-args -p interface_type:=bimanual -p config_path:=/workspace/libs/robot_motion_interface/config/bimanual_arm_config.yaml

# Launch simulation
ros2 run robot_motion_interface_ros interface --ros-args -p interface_type:=isaacsim -p config_path:=/workspace/libs/robot_motion_interface/config/isaacsim_config.yaml
```



2. In another terminal, launch the gamepad nodes:
```bash
source libs/primitives/ros/install/setup.bash
ros2 launch primitives_ros primitive_gamepad.launch.py
```
> If you're using docker, this one will need to be launched in the `compose.ros.gamepadx.yaml` Docker



