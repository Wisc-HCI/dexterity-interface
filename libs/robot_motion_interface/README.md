# Robot Motion Interface
Interface for Panda, Tesollo, isaacsim. Can be extended to more robots. Contains unified ROS interface for all robots.

## Requirements
* Ubuntu Machine. Since this is a C++ library, it should work with other operating systems, but the install instructions are only made for Ubuntu machines.
* robot_motion installed on machine. TODO: futher instructions

## Setup
1. Install Ubuntu Dependencies:
    ```bash
    sudo apt update
    sudo apt install libeigen3-dev
    ```

2. Build the cpp package(s)
    Make sure you are in the `robot_motion_interface_cpp` directory before running these commands:
    ```bash
    cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DBUILD_EXAMPLES=ON
    cmake --build build --parallel
    ```
3. Build python package(s):
TODO:

## Running
* Panda: TODO: REVISE AFTER TESTING
    ```bash
    ./build/interface_demo
    ```
TODO

### Isaacsim Utils
Make sure to run these in the root directory of `dexterity_interface`

* URDF to USD converter. See robot_motion_interface/isaacsim/utils/urdf_converter.py for documentation for full list parameters.

    ```bash
    python3 -m robot_motion_interface.isaacsim.utils.urdf_converter \
        path/to/robot.urdf path/to/out/robot.usd \
        --fix-base --joint-stiffness 0.0 --joint-damping 0.0 --joint-target-type none 
    ```

    **EXAMPLE:** <br>
    To convert the files in the project, first source the directories:
    ```bash
    export DESC=$(pwd)/libs/robot_description
    export SIM=$(pwd)/libs/robot_motion_interface/robot_motion_interface_py/src/robot_motion_interface_py/isaacsim
    mkdir -p $DESC/composites/tmp
    ```

    Then you can run any of the following to do xacro -> urdf -> usd.
    
    Panda arm with the gripper/hand:
    ```bash
    xacro $DESC/panda/panda_w_hand.urdf.xacro \
    file_prefix:="$DESC/panda" \
    name_prefix:="robot_" \
    -o  $DESC/composites/tmp/panda_w_hand.urdf

    python3 -m robot_motion_interface.isaacsim.utils.urdf_converter  \
    $DESC/composites/tmp/panda_w_hand.urdf \
    $SIM/usds/panda_w_hand/panda_w_hand.usd \
    --fix-base --joint-stiffness 0.0 --joint-damping 0.0
    ```



    Panda arm with the force torque sensor and the kinect:

    ```bash
    xacro $DESC/composites/panda_w_ft_kinect.urdf.xacro \
    panda_file_prefix:="$DESC/panda" \
    kinect_file_prefix:="$DESC/kinect" \
    ft_sensor_file_prefix:="$DESC/ft_sensor" \
    name_prefix:="robot_" \
    -o  $DESC/composites/tmp/panda_w_ft_kinect.urdf

    python3 -m robot_motion_interface.isaacsim.utils.urdf_converter  \
    $DESC/composites/tmp/panda_w_ft_kinect.urdf \
    $SIM/usds/panda_w_ft_kinect/panda_w_ft_kinect.usd \
    --fix-base --joint-stiffness 0.0 --joint-damping 0.0
    ```

    Panda arm with the tesollo gripper:

    ```bash
    xacro $DESC/composites/panda_w_tesollo.urdf.xacro \
    name_prefix:="robot_" \
    panda_file_prefix:="$DESC/panda" \
    tesollo_DG3F_file_prefix:="$DESC/tesollo_DG3F" \
    -o  $DESC/composites/tmp/panda_w_tesollo.urdf

    python3 -m robot_motion_interface.isaacsim.utils.urdf_converter  \
    $DESC/composites/tmp/panda_w_tesollo.urdf \
    $SIM/usds/panda_w_tesollo/panda_w_tesollo.usd \
    --fix-base --joint-stiffness 0.0 --joint-damping 0.0
    ```