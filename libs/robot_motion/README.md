# Robot Motion
C++ Package for common low-level robot motion functionality, like controllers, robot properties, and IK.

## Requirements
* Ubuntu Machine. Since this is a C++ and Python library, it should work with other operating systems, but the install instructions are only made for Ubuntu machines.

## Setup
1. Install Ubuntu Dependencies:
    ```bash
    sudo apt update
    sudo apt install python3.11-dev libeigen3-dev pybind11-dev
    ```
2. Follow [these instructions](https://stack-of-tasks.github.io/pinocchio/download.html) to install Pinnochio. Make sure to the add the environmental varialbes to your `~/.bashrc` and then re-source your bash (`source ~/.bashrc`).

3. Install rust using [these instructions](https://rust-lang.org/learn/get-started/). Cargo will also be installed by default.
4. Install Ranged IK:
    a. [click here](https://github.com/Wisc-HCI/relaxed_ik_core) and clone our fork of relaxed_ik_core to `~/robot-libs`.
    b. Inside  `~/robot-libs/relaxed_ik_core` run the following to build the package:
    ```bash
    cargo build
    ```
    c. Test that the install was successful by running `cargo run --bin relaxed_ik_bin`. It should output joint solutions.


5. Build C++ and Python packages. This installs the the python portion as a pip package and the C++ package to /usr/local by default. Make sure you are in the `robot_motion_cpp` directory before running these commands:

    ```bash
    # Python package (and C++ wrapper) install
    pip install -e .        
    
    # System C++ install to /usr/local
    cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
    cmake --build build -j
    sudo cmake --install build 
    ```

    You can test that the python wrappers were properly built by running `python -c "import robot_motion; print('OK')"`

# Running
To run the cpp joint torque controller example, run the following (this assumes you have `libs/robot_description`):
```bash
./build/joint_torque
```

To run the python joint torque example run the following:
```bash
python3 -m robot_motion.examples.joint_torque
```

To run the IK example, run:
```bash
python3 -m robot_motion.examples.bimanual_ik
```