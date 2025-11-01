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

3. [click here](https://github.com/uwgraphics/relaxed_ik_ros1/tree/ranged-ik#) and clone RangedIK into your `~/robot-libs` (you should also see Pinnochio here). Afterwards, create go into your RangedIK folder and then create an __init__.py file. Within the file, please put in
```from pathlib import Path
import sys

# Add relaxed_ik_core to Python path
module_path = Path(__file__).parent / "relaxed_ik_core"
sys.path.append(str(module_path))

# Now you can access core modules
import relaxed_ik_core
```
add `export PYTHONPATH=~/robot-libs/ranged_ik:$PYTHONPATH` to your `~/.bashrc`

4. Now in libs/robot_motion/src/robot_motion/ik/python_wrapper.py change the location of librelaxed_ik_lib.so to where ever this is.

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

current code makes you have to run ranged_ik in `/dexterity-interface/libs/robot_motion/src/robot_motion/ik`