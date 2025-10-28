1. [click here](https://github.com/uwgraphics/relaxed_ik_ros1/tree/ranged-ik#) and clone RangedIK into your `~/robot-libs`. 


    Currently not sure if need this instruction, but including anyways just in case. Afterwards, create go into your RangedIK folder and then create an __init__.py file. Within the file, please put in
   
    ```from pathlib import Path
    import sys

    # Add relaxed_ik_core to Python path
    module_path = Path(__file__).parent / "relaxed_ik_core"
    sys.path.append(str(module_path))

    # Now you can access core modules
    import relaxed_ik_core
    ```
    add `export PYTHONPATH=~/robot-libs/ranged_ik:$PYTHONPATH` to your `~/.bashrc`

2. Now in libs/robot_motion/src/robot_motion/ik/python_wrapper.py change the location of librelaxed_ik_lib.so to where ever this is on your machine


3. Now, Make sure you are in `/dexterity-interface/libs/robot_motion/src/robot_motion/ik`
before running anything

4. There are two programs currently, one for only wrist and one for wrist and finger tips for one arm. Look at teh settings.yaml and make sure you are using correct settings. Just commment out the setting that you are not using.

    4a. For only wrist, first ensure the `settings.yaml` in the `/ik/` folder only has one "chain", then do `python3 single_chain_ranged_ik.py`

    4b. For only wrist and finger, first ensure the `settings.yaml` in the `/ik/` folder only has one chain for wrist and 3 for fingers, then do `python3 multi_chain_ranged_ik.py`

5. (Optional) If want to visualize, update the settings file `starting_config` to what you get from the IK solver for a given target_pose in either single_chain_ranged_ik.py or multi_chain_ranged_ik.py
 Then `source run_bash_helper.sh` , which colcon builds rviz. Then ensure make sure to press Add > Robot Model. Then in that robot model, choose Description Source > Topic and Description Topic > /robot_description. Also ensure to update FixedFrame to one of the links. I recommend Table_Top