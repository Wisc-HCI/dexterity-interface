# USD Sources
* bimanual_arms: Generated from the urdfs in this robot_description (see below instructions)
* bowl: Onshape assembly [from here](https://cad.onshape.com/documents/6cc169caeb31cd08d910d204/w/0b283636293523a797a9c429/e/39f8f41ec4da2fc820945c39?renderMode=0&uiState=693adc0720c752a04de73d27) 
* Cup: Onshape assembly [from here](https://cad.onshape.com/documents/4c47262372500a49d0c2832c/w/f787994e7a0c037329aecba9/e/392e89385a5bb7985177a1dd?renderMode=0&uiState=693ad7756e9af32341a9dba0) 
* Funnel: Onshape assembly [from here](https://cad.onshape.com/documents/d19516ade0bd4f9b75b36c79/w/148b417dd4a60ee78b1c0e7a/e/fc6293d880b389c65e32171c?renderMode=0&uiState=6971447d64b693f60a8b9257)


## URDF to USD Conversion
To convert urdfs to usds, run the following instruction in the root directory of `dexterity_interface`:
```bash
python3 -m robot_motion_interface.isaacsim.utils.urdf_converter \
    path/to/robot.urdf path/to/out/robot.usd \
    --fix-base --joint-stiffness 0.0 --joint-damping 0.0 --joint-target-type none 
```

> See robot_motion_interface/isaacsim/utils/urdf_converter.py for documentation for full list parameters and installation requirements

**Bimanual Arm Conversion** <br>
Note: this requires the `robot_description` dependency.

To convert convert the bimanual arm setup to usd (xacro -> urdf -> usd), run the following: 
```bash
# Setup proper directories
export DESC=$(pwd)/libs/robot_description/ros/src/robot_description/urdf
export SIM=$(pwd)/libs/robot_motion_interface/src/robot_motion_interface/isaacsim
mkdir -p $DESC/composites/tmp

# Convert to urdf
xacro $DESC/composites/bimanual_arms.urdf.xacro \
    composite_file_prefix:="$DESC/composites" \
    panda_file_prefix:="$DESC/panda" \
    tesollo_DG3F_file_prefix:="$DESC/tesollo_DG3F" \
    -o  $DESC/composites/tmp/isaacsim_bimanual_arms.urdf

# Convert to usd
python3 -m robot_motion_interface.isaacsim.utils.urdf_converter  \
    $DESC/composites/tmp/isaacsim_bimanual_arms.urdf \
    $SIM/usds/bimanual_arms/bimanual_arms.usd \
    --fix-base --joint-stiffness 0.0 --joint-damping 0.0
```

> Note, once this is converted, you can import the USD again into isaacsim and follow step 6 below to adjust the friction of the gripper.

# Onshape Conversion
In order to convert the Onshape assemblies, follow these steps:
1. In your Onshape account, make sure you can open the onshape file.
2. Open Isaacsim and Select `File` > `Import from Onshape`. Authenticate Onshape when prompted.
3. In the Isaacsim Onshape Importer, double click the root of the object (i.e. bowl or cup).
4. In the table that shows up, choose PLA for the Material for each part. Then exit the popup.
5. Now add colliders:
    1. In the Isaacsim stage side-panel, right-click the root of the object and select `Add` > `Physics` > `Rigid Body with Colliders Preset`.
    2. Under the object, for EVERY part, click each one (should have child Looks folder and mesh) and in the Property tab and under `Physics` > `Collider` > `Approximation`, make sure `Convex Hull` is selected.
    3. Click the second child from the root and under the Property tab under `Physics` > `Articulation Root`, make sure `Articulation-Enabled` is un-selected.
6. Now add friction:
    1. In the Isaacsim stage side-panel, right-click anywhere and select `Create` > `Physics` > `Physics Material`. When the popup shows up, check `Rigid Body Material`.
    2. Click the newly generated material (should be named `PhysicsMaterial`). In the `Property Panel` under `Physics` > `Rigid Body Material`, set `Friction Combine Mode` to `Max`. Then enter the Dynamic Friction and Static Friction according to your desired material. Here are some guidelines:
    * Plastic: Static Friction=0.5, Dynamic Friction=0.4
    * Rubber: Static Friction=1.0, Dynamic Friction=0.8
    * Grippy Rubber: Static Friction=1.5, Dynamic Friction=1.4
    3. Again in the Stage side panel, under the object, for EVERY part, click each one (should have child Looks folder and mesh) and in the Property tab and under `Material on selected model`, in the field that says `None`, select the material (should be named `/Root/PhysicsMaterial`).
7. Then again in the Stage side panel, click the root object, then right click the root object and select Save Selected. Make sure to select .usd in the popup and save to your desired location.