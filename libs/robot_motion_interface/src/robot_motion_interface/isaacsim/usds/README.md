# USD Sources


* bimanual_arms: Generated from the urdfs in this repo (see robot_motion_interface/README.md)
* bowl: URDF from [this repo](https://gitlab.cs.washington.edu/ym2552/bullet3/-/tree/2.87/data/dinnerware) and converted to USD with this script: 
    ```bash
    # Make sure you are in this directory
    python3 -m robot_motion_interface.isaacsim.utils.urdf_converter converted_from_urdfs/bowl/bowl.urdf bowl/bowl.usd 
    ```
* Cup: STL made by Mya and converted with: 
    ```bash
    # Make sure you are in this directory
    python3 -m robot_motion_interface.isaacsim.utils.mesh_converter converted_from_meshes/Cup.stl  cup/cup.usd --mass 0.1
    ```