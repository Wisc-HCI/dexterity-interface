# Robot Motion Interface
Interface for Panda, Tesollo, isaacsim. Can be extended to more robots. Contains unified ROS interface for all robots.

## Requirements
* Ubuntu Machine. Since this is a C++ library, it should work with other operating systems, but the install instructions are only made for Ubuntu machines.
* robot_motion installed on machine. TODO: futher instructions

## C++ Setup
1. Install Ubuntu Dependencies:
    ```bash
    sudo apt update
    sudo apt install libeigen3-dev
    ```

2. Build the cpp package(s)
    Make sure you are in the `robot_motion_interface` directory before running these commands:
    ```bash
    cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
    cmake --build build -j
    ```

## C++ Running
For the example panda interface, run the following:
```bash
./build/panda
```


### Python Setup
```bash
pip install -e libs/robot_motion_interface
```

### Running Examples
```bash
python3 -m  robot_motion_interface.examples.oscillating_ex
python3 -m  robot_motion_interface.examples.static_ex
```

## Isaacsim Utils
Make sure to run these in the root directory of `dexterity_interface`

* URDF to USD converter. See robot_motion_interface/isaacsim/utils/urdf_converter.py for documentation for full list parameters and installation requirements.

    ```bash
    python3 -m robot_motion_interface.isaacsim.utils.urdf_converter \
        path/to/robot.urdf path/to/out/robot.usd \
        --fix-base --joint-stiffness 0.0 --joint-damping 0.0 --joint-target-type none 
    ```

    **EXAMPLE:** <br>
    To convert convert the bimanual arm setup to usd (xacro -> urdf -> usd), run the following: 
    ```bash
    # Setup proper directories
    export DESC=$(pwd)/libs/robot_description
    export SIM=$(pwd)/libs/robot_motion_interface/robot_motion_interface_py/src/robot_motion_interface/isaacsim
    mkdir -p $DESC/composites/tmp

    # Convert to urdf
    xacro $DESC/composites/bimanual_arms.urdf.xacro \
        composite_file_prefix:="$DESC/composites" \
        panda_file_prefix:="$DESC/panda" \
        tesollo_DG3F_file_prefix:="$DESC/tesollo_DG3F" \
        -o  $DESC/composites/tmp/bimanual_arms.urdf

    # Convert to usd
    python3 -m robot_motion_interface.isaacsim.utils.urdf_converter  \
        $DESC/composites/tmp/bimanual_arms.urdf \
        $SIM/usds/bimanual_arms/bimanual_arms.usd \
        --fix-base --joint-stiffness 0.0 --joint-damping 0.0
    ```

