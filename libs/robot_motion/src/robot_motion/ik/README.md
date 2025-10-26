
4. Now in libs/robot_motion/src/robot_motion/ik/python_wrapper.py change the location of librelaxed_ik_lib.so to where ever this is.


Make sure you are in /dexterity-interface/libs/robot_motion/src/robot_motion/ik before running anything

python3 ranged_ik.py # you can go into ranged_ik.py to change the target postion and orientations

^^ after you get the above, update settings.yaml's joint positions to reflect so. 

source run_bash_helper.sh # does the colcon build and starts the RVIz