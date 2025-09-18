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

    // Example for converting bimanual system
    python3 -m robot_motion_interface.isaacsim.utils.urdf_converter \
    libs/robot_description/bimanual_arms.urdf \
    libs/robot_motion_interface/robot_motion_interface_py/src/robot_motion_interface/isaacsim/usds/bimanual_arm/bimanual_arms.usd \
    --fix-base --joint-stiffness 0.0 --joint-damping 0.0 --joint-target-type none 


    // Example with Panda w/ kinect

    // Create urdf from usd
    export DESC=$(pwd)/libs/robot_description
    export SIM=$(pwd)/libs/robot_motion_interface/robot_motion_interface_py/src/robot_motion_interface/isaacsim/

    mkdir -p $DESC/composites/tmp

    xacro $DESC/composites/panda_with_kinect.urdf.xacro \
    kinect_file_prefix:="$DESC/kinect" \
    panda_file_prefix:="$DESC/panda" \
    -o  $DESC/composites/tmp/panda_with_kinect.urdf

    python3 -m robot_motion_interface.isaacsim.utils.urdf_converter  \
    $DESC/composites/tmp/panda_with_kinect.urdf \
    $SIM/usds/panda_with_kinect/panda_with_kinect.usd \
    --fix-base --joint-stiffness 0.0 --joint-damping 0.0
    ```


```bash

xacro $DESC/panda/panda_w_hand.urdf.xacro \
panda_file_prefix:="$DESC/panda" \
-o  $DESC/composites/tmp/panda_w_hand.urdf

python3 -m robot_motion_interface.isaacsim.utils.urdf_converter  \
$DESC/composites/tmp/panda_w_hand.urdf \
$SIM/usds/panda_with_kinect/panda_with_kinect.usd \
--fix-base --joint-stiffness 0.0 --joint-damping 0.0
```


```bash

xacro $DESC/panda/panda_hand.urdf.xacro \
panda_file_prefix:="$DESC/panda" \
-o  $DESC/composites/tmp/panda_hand.urdf

python3 -m robot_motion_interface.isaacsim.utils.urdf_converter  \
$DESC/composites/tmp/panda_hand.urdf \
$SIM/usds/panda_with_kinect/panda_with_kinect.usd \
--fix-base --joint-stiffness 0.0 --joint-damping 0.0
```