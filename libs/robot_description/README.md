# Robot Description

## ROS Installation
Make sure you are in the robot_description/ros directory.
```bash
colcon build --symlink-install
source install/setup.sh
```

## ROS Running
To launch the rviz node, run the following:
```bash
ros2 launch robot_description newdisplay.launch.py urdf_path:=[absolute path to urdf] rviz_config:=[absolute path to rviz config]
```
Test publishing a joint state and see it update the rviz (robot description defaults to listening at /joint_state)
```bash
ros2 topic pub -r 10 /joint_states sensor_msgs/msg/JointState \
"{header: {stamp: 'now', frame_id: ''},
  name: [
    'left_panda_joint1','left_panda_joint2','left_panda_joint3','left_panda_joint4','left_panda_joint5','left_panda_joint6','left_panda_joint7',
    'left_F1M1','left_F1M2','left_F1M3','left_F1M4','left_F2M1','left_F2M2','left_F2M3','left_F2M4', 'left_F3M1','left_F3M2','left_F3M3','left_F3M4',
    'right_panda_joint1','right_panda_joint2','right_panda_joint3','right_panda_joint4','right_panda_joint5','right_panda_joint6','right_panda_joint7',
    'right_F1M1','right_F1M2','right_F1M3','right_F1M4', 'right_F2M1','right_F2M2','right_F2M3','right_F2M4', 'right_F3M1','right_F3M2','right_F3M3','right_F3M4'
  ],
  position: [
    0.0, -0.7854, 0.0, -2.3562, 0.0, 1.5708, 0.7854,             
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  
    0.0, -0.7854, 0.0, -2.3562, 0.0, 1.5708, 0.7854,            
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0  
  ]}"

```

TODO: Fix joint name in tesollo urdf

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


