# USD Sources


* bimanual_arms: Generated from the urdfs in this repo (see robot_motion_interface/README.md)
* Bowl: STL made by in [Onshape](https://cad.onshape.com/documents/6cc169caeb31cd08d910d204/w/0b283636293523a797a9c429/e/39f8f41ec4da2fc820945c39?renderMode=0&uiState=693adc0720c752a04de73d27) and converted with: 
    ```bash
    # Make sure you are in this directory
    python3 -m robot_motion_interface.isaacsim.utils.mesh_converter converted_from_meshes/Bowl.stl  bowl/bowl.usd --mass 0.1
    ```
* Cup: STL made by in [Onshape](https://cad.onshape.com/documents/4c47262372500a49d0c2832c/w/f787994e7a0c037329aecba9/e/392e89385a5bb7985177a1dd?renderMode=0&uiState=693ad7756e9af32341a9dba0) and converted with: 
    ```bash
    # Make sure you are in this directory
    python3 -m robot_motion_interface.isaacsim.utils.mesh_converter converted_from_meshes/Cup.stl  cup/cup.usd --mass 0.1
    ```