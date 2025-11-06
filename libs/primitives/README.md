# Primitives

## Setup
1. This package depends on `robot_motion`, `robot_motion_interface`, and ROS 2 Jazzy. To install all of these dependencies build the docker container (instructions in the root readme) and run the following in the the container. 

2. Compile robot_motion_interface_ros and this package. Make sure you are in the `libs/robot_motion/ros` directory before running these. 
```bash
cd libs/robot_motion_interface/ros
colcon build --symlink-install

cd ../../../libs/primitives/ros
colcon build --symlink-install
cd ../../..
```


## Running
1. In one terminal launch either the real or simulated interface (or both)
```bash
source libs/robot_motion_interface/ros/install/setup.bash
# Launch Real
ros2 run robot_motion_interface_ros interface --ros-args -p interface_type:=bimanual -p config_path:=/workspace/libs/robot_motion_interface/config/bimanual_arm_config.yaml

# Launch simulation
ros2 run robot_motion_interface_ros interface --ros-args -p interface_type:=isaacsim -p config_path:=/workspace/libs/robot_motion_interface/config/isaacsim_config.yaml
```

2. In another terminal, launch the primitive node:
```bash
source libs/primitives/ros/install/setup.bash
ros2 run primitives_ros primitive_handler
```

Now, here are some topics you can test publishing to
```bash
source libs/primitives/ros/install/setup.bash
# Grasp with left gripper
ros2 topic pub --once /primitive/envelop_grasp std_msgs/msg/String "{data: 'left'}" 

# Grasp with right gripper
ros2 topic pub --once /primitive/envelop_grasp std_msgs/msg/String "{data: 'right'}" 

# Release with left gripper
ros2 topic pub --once /primitive/release std_msgs/msg/String "{data: 'left'}" 

# Release with right gripper
ros2 topic pub --once /primitive/release std_msgs/msg/String "{data: 'right'}" 

# Move left robot
ros2 topic pub /primitive/move_to_pose geometry_msgs/PoseStamped "{ header: {frame_id: 'left'}, pose: {position: {x: -0.2, y: 0.2, z: 0.4}, orientation: {x: 0.707, y: 0.707, z: 0.0, w: 0.0} }}" --once

# Move right robot
ros2 topic pub /primitive/move_to_pose geometry_msgs/PoseStamped "{ header: {frame_id: 'right'}, pose: {position: {x: 0.2, y: 0.2, z: 0.4}, orientation: {x: 0.707, y: 0.707, z: 0.0, w: 0.0} }}" --once
```


## Running Joy example
If you have a joystick controller (xbox controller), you can connect it via usb or bluetooth. Then you can teleop the robot with it.

Make sure you have joy ROS package installed (`sudo apt install ros-jazzy-joy`).

Then run both nodes in the prior section (#1, #2). After that launch these 2 nodes in seperate terminals:
```bash
ros2 run joy joy_node
source libs/primitives/ros/install/setup.bash
ros2 run primitives_ros joy_handler
```

TODO: Launch file for these, better home position



# DEBUGGING: TODO: DELETE
SET q: [0. 0. 0. 0. 0. 0. 0. 0. 0. 0. 0. 0.] | JOINT NAMES: None
SET q: [ 0.     -0.7854  0.     -2.3562  0.      1.5708  0.7854] | JOINT NAMES: None
SET q: [0. 0. 0. 0. 0. 0. 0. 0. 0. 0. 0. 0.] | JOINT NAMES: None
SET q: [ 0.     -0.7854  0.     -2.3562  0.      1.5708  0.7854] | JOINT NAMES: None
SET q: [ 0.62877044 -0.73060356 -0.66219139 -2.19229279 -0.44966984  1.59838148
 -1.39995722 -0.04056521 -0.76161683  0.00611724 -2.40599959 -0.02814348
  1.57917424  0.84439772] | JOINT NAMES: ['left_panda_joint1', 'left_panda_joint2', 'left_panda_joint3', 'left_panda_joint4', 'left_panda_joint5', 'left_panda_joint6', 'left_panda_joint7', 'right_panda_joint1', 'right_panda_joint2', 'right_panda_joint3', 'right_panda_joint4', 'right_panda_joint5', 'right_panda_joint6', 'right_panda_joint7']
<HERE RELEASE primitive was used>
CUR q: [ 3.22352387e-02 -7.67347147e-01 -7.89346656e-03 -2.35800227e+00
 -3.04972086e-02  1.49858166e+00  6.47907310e-01 -5.23598776e-03
 -1.74532925e-03  1.06465084e-01  3.66519143e-02 -1.74532925e-03
  0.00000000e+00  9.94837674e-02  1.91986218e-02  3.49065850e-03
 -3.49065850e-03  9.94837674e-02  1.39626340e-02 -1.99573438e-02
 -7.59377802e-01 -6.25352545e-03 -2.40605326e+00 -3.74409710e-02
  1.58075144e+00  8.58564051e-01 -3.49065850e-03  0.00000000e+00
  1.22173048e-01  3.14159265e-02 -1.74532925e-03 -1.74532925e-03
  1.02974426e-01  5.23598776e-03  3.49065850e-03 -3.49065850e-03
  1.32645023e-01  5.23598776e-03]
FULL q: [ 6.28770440e-01 -7.30603557e-01 -6.62191393e-01 -2.19229279e+00
 -4.49669842e-01  1.59838148e+00 -1.39995722e+00 -5.23598776e-03
 -1.74532925e-03  1.06465084e-01  3.66519143e-02 -1.74532925e-03
  0.00000000e+00  9.94837674e-02  1.91986218e-02  3.49065850e-03
 -3.49065850e-03  9.94837674e-02  1.39626340e-02 -4.05652138e-02
 -7.61616830e-01  6.11724336e-03 -2.40599959e+00 -2.81434827e-02
  1.57917424e+00  8.44397724e-01 -3.49065850e-03  0.00000000e+00
  1.22173048e-01  3.14159265e-02 -1.74532925e-03 -1.74532925e-03
  1.02974426e-01  5.23598776e-03  3.49065850e-03 -3.49065850e-03
  1.32645023e-01  5.23598776e-03]
__________________________________
SET q: [ 0.62877044 -0.73060356 -0.66219139 -2.19229279 -0.44966984  1.59838148
 -1.39995722] | JOINT NAMES: None
SET q: [-0.00523599 -0.00174533  0.10646508  0.03665191 -0.00174533  0.
  0.09948377  0.01919862  0.00349066 -0.00349066  0.09948377  0.01396263] | JOINT NAMES: None
SET q: [-0.04056521 -0.76161683  0.00611724 -2.40599959 -0.02814348  1.57917424
  0.84439772] | JOINT NAMES: None
SET q: [-0.00349066  0.          0.12217305  0.03141593 -0.00174533 -0.00174533
  0.10297443  0.00523599  0.00349066 -0.00349066  0.13264502  0.00523599] | JOINT NAMES: None
SET q: [ 0.71519165 -0.61718641 -0.83045899 -2.03526696 -0.42828766  1.57534318
 -1.61349214 -0.04114495 -0.75999246  0.00910724 -2.40779549 -0.02830715
  1.58254081  0.84876757] | JOINT NAMES: ['left_panda_joint1', 'left_panda_joint2', 'left_panda_joint3', 'left_panda_joint4', 'left_panda_joint5', 'left_panda_joint6', 'left_panda_joint7', 'right_panda_joint1', 'right_panda_joint2', 'right_panda_joint3', 'right_panda_joint4', 'right_panda_joint5', 'right_panda_joint6', 'right_panda_joint7']
CUR q: [ 3.82921874e-02 -7.66702777e-01 -1.35440166e-02 -2.35689199e+00
 -4.09979716e-02  1.49880093e+00  6.31287177e-01  0.00000000e+00
 -3.49065850e-03  1.08210414e-01  3.66519143e-02  0.00000000e+00
 -1.74532925e-03  1.01229097e-01  1.91986218e-02  5.23598776e-03
 -3.49065850e-03  9.94837674e-02  1.39626340e-02 -2.00142783e-02
 -7.59359559e-01 -6.20300442e-03 -2.40609495e+00 -3.74445021e-02
  1.58079968e+00  8.58557253e-01 -3.49065850e-03  0.00000000e+00
  1.22173048e-01  3.14159265e-02 -1.74532925e-03 -1.74532925e-03
  1.02974426e-01  5.23598776e-03  3.49065850e-03 -3.49065850e-03
  1.34390352e-01  5.23598776e-03]
FULL q: [ 7.15191653e-01 -6.17186405e-01 -8.30458995e-01 -2.03526696e+00
 -4.28287659e-01  1.57534318e+00 -1.61349214e+00  0.00000000e+00
 -3.49065850e-03  1.08210414e-01  3.66519143e-02  0.00000000e+00
 -1.74532925e-03  1.01229097e-01  1.91986218e-02  5.23598776e-03
 -3.49065850e-03  9.94837674e-02  1.39626340e-02 -4.11449467e-02
 -7.59992464e-01  9.10723929e-03 -2.40779549e+00 -2.83071464e-02
  1.58254081e+00  8.48767572e-01 -3.49065850e-03  0.00000000e+00
  1.22173048e-01  3.14159265e-02 -1.74532925e-03 -1.74532925e-03
  1.02974426e-01  5.23598776e-03  3.49065850e-03 -3.49065850e-03
  1.34390352e-01  5.23598776e-03]
__________________________________
SET q: [ 0.71519165 -0.61718641 -0.83045899 -2.03526696 -0.42828766  1.57534318
 -1.61349214] | JOINT NAMES: None
SET q: [ 0.         -0.00349066  0.10821041  0.03665191  0.         -0.00174533
  0.1012291   0.01919862  0.00523599 -0.00349066  0.09948377  0.01396263] | JOINT NAMES: None
SET q: [-0.04114495 -0.75999246  0.00910724 -2.40779549 -0.02830715  1.58254081
  0.84876757] | JOINT NAMES: None
SET q: [-0.00349066  0.          0.12217305  0.03141593 -0.00174533 -0.00174533
  0.10297443  0.00523599  0.00349066 -0.00349066  0.13439035  0.00523599] | JOINT NAMES: None
SET q: [ 0.69523502 -0.63329183 -0.77847666 -2.06373524 -0.42527276  1.57307024
 -1.54056122 -0.04132078 -0.75907902  0.00812406 -2.4067608  -0.02841923
  1.58132142  0.84766571] | JOINT NAMES: ['left_panda_joint1', 'left_panda_joint2', 'left_panda_joint3', 'left_panda_joint4', 'left_panda_joint5', 'left_panda_joint6', 'left_panda_joint7', 'right_panda_joint1', 'right_panda_joint2', 'right_panda_joint3', 'right_panda_joint4', 'right_panda_joint5', 'right_panda_joint6', 'right_panda_joint7']
CUR q: [ 1.29545332e-01 -7.57600587e-01 -8.86499462e-02 -2.33819913e+00
 -1.26811907e-01  1.50021195e+00  4.73589295e-01 -8.72664626e-03
 -1.74532925e-03  1.20427718e-01  3.83972435e-02  0.00000000e+00
 -1.74532925e-03  1.09955743e-01  2.09439510e-02  1.74532925e-03
 -5.23598776e-03  1.08210414e-01  1.74532925e-02 -2.15014555e-02
 -7.59034652e-01 -5.98897521e-03 -2.40710330e+00 -3.75189431e-02
  1.58105735e+00  8.58560960e-01 -1.74532925e-03  0.00000000e+00
  1.29154365e-01  3.31612558e-02 -1.74532925e-03 -1.74532925e-03
  1.11701072e-01  8.72664626e-03  3.49065850e-03 -3.49065850e-03
  1.37881011e-01  5.23598776e-03]
FULL q: [ 6.95235019e-01 -6.33291832e-01 -7.78476658e-01 -2.06373524e+00
 -4.25272761e-01  1.57307024e+00 -1.54056122e+00 -8.72664626e-03
 -1.74532925e-03  1.20427718e-01  3.83972435e-02  0.00000000e+00
 -1.74532925e-03  1.09955743e-01  2.09439510e-02  1.74532925e-03
 -5.23598776e-03  1.08210414e-01  1.74532925e-02 -4.13207789e-02
 -7.59079023e-01  8.12405963e-03 -2.40676080e+00 -2.84192296e-02
  1.58132142e+00  8.47665714e-01 -1.74532925e-03  0.00000000e+00
  1.29154365e-01  3.31612558e-02 -1.74532925e-03 -1.74532925e-03
  1.11701072e-01  8.72664626e-03  3.49065850e-03 -3.49065850e-03
  1.37881011e-01  5.23598776e-03]
__________________________________
SET q: [ 0.69523502 -0.63329183 -0.77847666 -2.06373524 -0.42527276  1.57307024
 -1.54056122] | JOINT NAMES: None
SET q: [-0.00872665 -0.00174533  0.12042772  0.03839724  0.         -0.00174533
  0.10995574  0.02094395  0.00174533 -0.00523599  0.10821041  0.01745329] | JOINT NAMES: None
SET q: [-0.04132078 -0.75907902  0.00812406 -2.4067608  -0.02841923  1.58132142
  0.84766571] | JOINT NAMES: None
SET q: [-0.00174533  0.          0.12915436  0.03316126 -0.00174533 -0.00174533
  0.11170107  0.00872665  0.00349066 -0.00349066  0.13788101  0.00523599] | JOINT NAMES: None
SET q: [ 0.68279265 -0.65576015 -0.75717933 -2.09062425 -0.43330087  1.57846398
 -1.52337403 -0.0418973  -0.75842093  0.00781284 -2.40737171 -0.02888191
  1.58126029  0.8479771 ] | JOINT NAMES: ['left_panda_joint1', 'left_panda_joint2', 'left_panda_joint3', 'left_panda_joint4', 'left_panda_joint5', 'left_panda_joint6', 'left_panda_joint7', 'right_panda_joint1', 'right_panda_joint2', 'right_panda_joint3', 'right_panda_joint4', 'right_panda_joint5', 'right_panda_joint6', 'right_panda_joint7']
CUR q: [ 2.09793468e-01 -7.45181033e-01 -1.74722545e-01 -2.30787821e+00
 -1.82438327e-01  1.50060589e+00  3.41938172e-01 -6.98131701e-03
  0.00000000e+00  1.23918377e-01  3.83972435e-02  0.00000000e+00
  1.74532925e-03  1.11701072e-01  2.26892803e-02  6.98131701e-03
 -6.98131701e-03  1.09955743e-01  1.74532925e-02 -2.25104640e-02
 -7.58594320e-01 -5.55386514e-03 -2.40788469e+00 -3.75258409e-02
  1.58120394e+00  8.58562181e-01 -5.23598776e-03  0.00000000e+00
  1.30899694e-01  3.31612558e-02 -1.74532925e-03 -1.74532925e-03
  1.11701072e-01  8.72664626e-03  5.23598776e-03 -3.49065850e-03
  1.41371669e-01  6.98131701e-03]
FULL q: [ 6.82792655e-01 -6.55760155e-01 -7.57179330e-01 -2.09062425e+00
 -4.33300871e-01  1.57846398e+00 -1.52337403e+00 -6.98131701e-03
  0.00000000e+00  1.23918377e-01  3.83972435e-02  0.00000000e+00
  1.74532925e-03  1.11701072e-01  2.26892803e-02  6.98131701e-03
 -6.98131701e-03  1.09955743e-01  1.74532925e-02 -4.18973032e-02
 -7.58420928e-01  7.81284324e-03 -2.40737171e+00 -2.88819078e-02
  1.58126029e+00  8.47977095e-01 -5.23598776e-03  0.00000000e+00
  1.30899694e-01  3.31612558e-02 -1.74532925e-03 -1.74532925e-03
  1.11701072e-01  8.72664626e-03  5.23598776e-03 -3.49065850e-03
  1.41371669e-01  6.98131701e-03]
__________________________________
