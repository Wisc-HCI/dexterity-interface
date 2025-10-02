# Robot Description


## Notes
TODO: Make this into python package??

If you want to convert xacro to urdf (for isaacsim, for example), you can do the following:
#### Installation
```bash
pip install xacro
```
#### Running
Make sure you are in this root director (`robot_description`). Then run:
```bash
xacro robot.urdf.xacro -o composities/urdfsrobot.urdf
```



**EXAMPLES:** <br>
1. First setup your paths:
    ```bash
    export DESC=$(pwd)/libs/robot_description
    mkdir -p $DESC/composites/tmp
    ```
2. Then you can run any of the following to do xacro -> urdf.
  
    Panda arm with the gripper/hand:
    ```bash
    xacro $DESC/panda/panda_w_hand.urdf.xacro \
        file_prefix:="$DESC/panda" \
        name_prefix:="robot_" \
        -o  $DESC/composites/tmp/panda_w_hand.urdf
    ```

    Panda arm with the force torque sensor and the kinect:

    ```bash
    xacro $DESC/composites/panda_w_ft_kinect.urdf.xacro \
        panda_file_prefix:="$DESC/panda" \
        kinect_file_prefix:="$DESC/kinect" \
        ft_sensor_file_prefix:="$DESC/ft_sensor" \
        name_prefix:="robot_" \
        -o  $DESC/composites/tmp/panda_w_ft_kinect.urdf
    ```

    Panda arm with the tesollo gripper:

    ```bash
    xacro $DESC/composites/panda_w_tesollo.urdf.xacro \
        standalone:="true" \
        name_prefix:="robot_" \
        panda_file_prefix:="$DESC/panda" \
        tesollo_DG3F_file_prefix:="$DESC/tesollo_DG3F" \
        -o  $DESC/composites/tmp/panda_w_tesollo.urdf
    ```

    Bimanual system:
    ```bash
    xacro $DESC/composites/bimanual_arms.urdf.xacro \
        composite_file_prefix:="$DESC/composites" \
        panda_file_prefix:="$DESC/panda" \
        tesollo_DG3F_file_prefix:="$DESC/tesollo_DG3F" \
        -o  $DESC/composites/tmp/bimanual_arms.urdf
    ```


