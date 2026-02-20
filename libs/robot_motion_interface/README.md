# Robot Motion Interface
Interface for Panda, Tesollo, isaacsim. Can be extended to more robots. Contains unified ROS interface for all robots.

## Requirements
* Ubuntu Machine. Since this is a C++ library, it should work with other operating systems, but the install instructions are only made for Ubuntu machines.
* robot_motion installed on machine. TODO: futher instructions


### Panda Requirements
* Ubuntu Machine with the [Real Time Kernel](https://frankaemika.github.io/docs/installation_linux.html#setting-up-the-real-time-kernel)
* Franka Emika Panda 7 DOF Robot setup with the [FCI](https://frankaemika.github.io/docs/getting_started.html).
	* Robot system version: 4.2.X (FER pandas). This is compatible with Libfranka version >= 0.9.1 < 0.10.0. We will use 0.9.2.

### Tesollo Requirements
* Tesollo 3 Finger Gripper (DG-3F) set to external mode with the switches. TODO: Add

# Setup Option 1
Follow the instructions in the root README to setup and run the docker container.


## Setup Option 2 
**NOTE: If you are running Isaacsim inside ROS (i.e. interface_node.py with Isaacsim), you WILL need to folllow Option 1 instead of this option.**

First setup the C++ dependencies:

1. Install Ubuntu Dependencies:
    ```bash
    sudo apt update
    sudo apt install libeigen3-dev
    ```
2. Install Libfranka. These instructions are adapted from [here](https://github.com/frankarobotics/libfranka):
    ```bash
    # Prep dependencies
    sudo apt-get update
    sudo apt-get install -y build-essential cmake git libpoco-dev libeigen3-dev libfmt-dev
    sudo apt-get remove "*libfranka*"

    # Clone and setup repo
    git clone --recurse-submodules https://github.com/frankarobotics/libfranka.git
    cd libfranka
    git checkout 0.9.2
    git submodule update

    # Build
    mkdir build
    cd build
    cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH=/opt/openrobots/lib/cmake -DBUILD_TESTS=OFF ..
    make

    # Install as deb package
    cpack -G DEB
    sudo dpkg -i libfranka*.deb

    # Remove repo since no longer needed
    cd ../..
    rm -rf ./libfranka
    ```
3. Build the cpp package(s).
    Make sure you are in the `robot_motion_interface` directory before running these commands:
    ```bash
    cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
    cmake --build build -j
    ```


Next, install the python files and the Panda C++ wrappers.
1. Run the following (recommend doing this in venv venv-dex):
    ```bash
    pip install -e libs/robot_motion_interface
    ```

    You can test that the python wrappers were properly built by running `python -c "import robot_motion_interface; print('OK')"`

[OPTIONAL] If you are running any of the isaacsim examples, you will Additionally need to install Isaacsim (recommend doing this in venv venv-dex):
1. Install Ubuntu dependencies:
    ```bash
    sudo apt update
    sudo apt install libeigen3-dev python3.11 python3.11-venv
    ```

2. Install IsaacSim and IsaacLab. 
    ```bash
    pip install --upgrade pip
    pip install torch==2.7.0 torchvision==0.22.0 --index-url https://download.pytorch.org/whl/cu128
    pip install isaaclab[isaacsim,all]==2.2.0 --extra-index-url https://pypi.nvidia.com
    ```

    Test that Isaacsim installed correctly. The first time this is run, make sure to reply YES to the EULA prompt. Also the first time it may take a while to start as it sets up the necessary dependencies.
    ```bash
    isaacsim
    ```

[OPTIONAL] If you are running the ROS examples (i.e. interface_node.py), you will also need to install ROS and setup up the ROS package.
1. Follow [these instructions](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html) to install ROS Jazzy on Ubuntu.
2. TODO: Other ROS dependencies (see docker)


## C++ Examples
For the example panda interface, run the following:
```bash
./build/panda
```

For the example tesollo interface, run the following:
```bash
./build/tesollo
```


### Python Examples
TODO: CLEAN THESE UP and add more explanation
Make sure you are in the `libs/robot_motion_interface` directory before running these.
```bash
python3 -m  robot_motion_interface.examples.oscillating_ex_panda_tesollo
python3 -m  robot_motion_interface.examples.oscillating_ex --interface panda
python3 -m  robot_motion_interface.examples.oscillating_ex --interface isaacsim
python3 -m  robot_motion_interface.examples.isaacsim_static
python3 -m  robot_motion_interface.examples.isaacsim_cartesian
python3 -m  robot_motion_interface.examples.isaacsim_objects
python3 -m  robot_motion_interface.examples.isaacsim_blocking
```



## ROS Running

1. Make sure you are in the `libs/robot_motion_interface/ros` directory. Then build the package:
    ```bash
    colcon build --symlink-install
    source install/setup.bash
    ```
    2. Now you can run any of these:
    ```bash
    # Launch bimanual arms
    ros2 run robot_motion_interface_ros interface --ros-args -p interface_type:=bimanual -p config_path:=/workspace/libs/robot_motion_interface/config/bimanual_arm_config.yaml

    # Launch simulation
    ros2 run robot_motion_interface_ros interface --ros-args -p interface_type:=isaacsim -p config_path:=/workspace/libs/robot_motion_interface/config/isaacsim_config.yaml

    # Launch simulation with object interface
    ros2 run robot_motion_interface_ros interface --ros-args -p interface_type:=isaacsim_object -p config_path:=/workspace/libs/robot_motion_interface/config/isaacsim_config.yaml


    # Launch left Panda
    ros2 run robot_motion_interface_ros interface --ros-args -p interface_type:=panda -p config_path:=/workspace/libs/robot_motion_interface/config/left_panda_config.yaml

    # Launch left Tesollo
    ros2 run robot_motion_interface_ros interface --ros-args -p interface_type:=tesollo -p config_path:=/workspace/libs/robot_motion_interface/config/left_tesollo_config.yaml
    ```


    Here are some topics you can publish to:
    ```bash

    # Home robot
    ros2 topic pub --once /home std_msgs/msg/Empty "{}" 

    # Publish cartesian position to left panda
    ros2 topic pub /set_cartesian_pose geometry_msgs/PoseStamped "{ header: {frame_id: 'left_delto_offset_link'}, pose: {position: {x: -0.2, y: 0.2, z: 1.2}, orientation: {x: 0.707, y: 0.707, z: 0.0, w: 0.0} }}" --once

    # Publish cartesian position to right panda
    ros2 topic pub /set_cartesian_pose geometry_msgs/PoseStamped "{ header: {frame_id: 'right_delto_offset_link'}, pose: {position: {x: 0.2, y: 0.2, z: 1.2}, orientation: {x: 0.707, y: 0.707, z: 0.0, w: 0.0} }}" --once

    # Publish 12 joints to Tesollo
    ros2 topic pub /set_joint_state sensor_msgs/msg/JointState '{ name: ["left_F1M1", "left_F1M2", "left_F1M3", "left_F1M4", "left_F2M1", "left_F2M2", "left_F2M3", "left_F2M4", "left_F3M1", "left_F3M2", "left_F3M3", "left_F3M4", ], position: [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]}' --once

    # Partial Left Tesollo update
    ros2 topic pub /set_joint_state sensor_msgs/msg/JointState '{ name: ["left_F1M1"], position: [-0.1]}' --once

    # Partial Right Tesollo update
    ros2 topic pub /set_joint_state sensor_msgs/msg/JointState '{ name: [ "right_F1M3", "right_F1M4", "right_F2M3", "right_F2M4", "right_F3M3", "right_F3M4"], position: [1.5, 1.5, 1.5, 1.5, 1.5, 1.5]}' --once


    # Publish 7 joints to left Panda
    ros2 topic pub /set_joint_state sensor_msgs/msg/JointState '{ name: ["left_panda_joint1", "left_panda_joint2", "left_panda_joint3", "left_panda_joint4", "left_panda_joint5" ,"left_panda_joint6", "left_panda_joint7"], position: [0.00, -1.05, 0.0, -2.36, 0.0, 1.57, 0.79]}' --once

    # Publish 7 joints to right Panda
    ros2 topic pub /set_joint_state sensor_msgs/msg/JointState '{ name: ["right_panda_joint1", "right_panda_joint2", "right_panda_joint3", "right_panda_joint4", 
            "right_panda_joint5" ,"right_panda_joint6", "right_panda_joint7"], position: [0.00, -1.05, 0.0, -2.36, 0.0, 1.57, 0.79]}' --once
    ```

    Here are some actions you can publish:
    ```bash
    # Move left arm
    ros2 action send_goal /set_cartesian_pose robot_motion_interface_ros_msgs/action/SetCartesianPose "pose_stamped: { header: {frame_id: 'left_delto_offset_link'}, pose: {position: {x: -0.2, y: 0.2, z: 1.2}, orientation: {x: 0.707, y: 0.707, z: 0.0, w: 0.0}} }"

    # Move right arm to another position
    ros2 action send_goal /set_cartesian_pose robot_motion_interface_ros_msgs/action/SetCartesianPose "pose_stamped: { header: {frame_id: 'right_delto_offset_link'}, pose: {position: {x: -0.1, y: 0.4, z: 0.2}, orientation: {x: 0.707, y: 0.707, z: 0.0, w: 0.0}} }"

    # Partial Right Tesollo joint update
    ros2 action send_goal /set_joint_positions robot_motion_interface_ros_msgs/action/SetJointPositions 'joint_state: { name: [ "right_F1M3", "right_F1M4", "right_F2M3", "right_F2M4", "right_F3M3", "right_F3M4"], position: [1.5, 1.5, 1.5, 1.5, 1.5, 1.5]}'

    # Home
    ros2 action send_goal /home robot_motion_interface_ros_msgs/action/Home "{}"

    # Cancel Homeing goal
    ros2 service call home/_action/cancel_goal action_msgs/srv/CancelGoal
    ```
    If you are running the isaacsim_object interface, you can additionally run these:
    ```bash
    # 1. Spawn bowl (other options: cup, cube, sphere, cylinder)
    ros2 topic pub /spawn_object geometry_msgs/PoseStamped "
    header: {frame_id: 'bowl'}
    pose:
    position: {x: 0.3, y: -0.2, z: 0.95}
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}" --once

    # 2. Move bowl
    ros2 topic pub /move_object geometry_msgs/PoseStamped "
    header: {frame_id: 'bowl'}
    pose:
    position: {x: 0.4, y: 0.2, z: 0.95}
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}" --once

    ```

