# Robot Motion
C++ Package for common low-level robot motion functionality, like controllers, robot properties, and IK.

## Requirements
* Ubuntu Machine. Since this is a C++ library, it should work with other operating systems, but the install instructions are only made for Ubuntu machines.

## Setup
1. Install Ubuntu Dependencies:
    ```bash
    sudo apt update
    sudo apt install libeigen3-dev
    ```

2. Build and Link this package:
    This installs the package to /usr/local by default. Make sure you are in the `robot_motion` directory before running these commands:
    ```bash
    cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
    cmake --build build --parallel
    sudo cmake --install build 
    ```
