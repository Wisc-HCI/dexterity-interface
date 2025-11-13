# Demo

These are the instructions for running the teleoporation demo for the bimanual system.

## Setup
### 1. Robot Setup
1. Turn on 2 Tesollo grippers by switching the switch on the power box.
2. Setup the pandas:
    a. Turn on the 2 Pandas by switching the switches on the back of each control box.
    b. In chrome on the labtop, navigate to `192.168.4.2` and `192.168.4.3` and click the unlock symbol on the right sidebar to unlock the joints on each interface.
    c. Once all the joints are unlocked, on both interfaces, click the hamburger icon in the upper right corner and select `Activate FCI`. Make sure the `FCI is active` popup shows up.

### 2. Gamepad Setup
1. Turn on the gamepad (8BitDo SN30 pro) by holding `+start` and `X` together until the lights on the bottom of the gamepad turn on.
2. Make sure that the gamepad is connected to the labtop via bluetooth in setting. It will also vibrate when connected.

If it is not connecting, use [this manual](https://support.8bitdo.com/Manual/sn30-pro/) and follow the instructions under `Windows` > `Bluetooth Connection` to pair the gamepad to the computer.


### 3. Code Setup
For this you will need to open 2 terminals. Run the following commands in each terminal

Terminal 1:
```bash
cd ~/Desktop/github/dexterity-interface
xhost +local:
sudo docker compose -f compose.ros.yaml build
sudo docker compose -f compose.ros.yaml -f compose.ros.gamepad.yaml run --rm ros-base

cd libs/robot_motion_interface/ros
colcon build --symlink-install
cd ../../..
source libs/robot_motion_interface/ros/install/setup.bash
```

Terminal 2:
```bash
cd ~/Desktop/github/dexterity-interface
sudo docker compose -f compose.ros.yaml -f compose.ros.gamepad.yaml exec ros-base bash

cd libs/primitives/ros
colcon build --symlink-install
cd ../../..
source libs/primitives/ros/install/setup.bash
```


## Running
Terminal 1:
```bash
ros2 run robot_motion_interface_ros interface --ros-args -p interface_type:=bimanual -p config_path:=/workspace/libs/robot_motion_interface/config/bimanual_arm_config.yaml
```

Terminal 2:
```bash
ros2 launch primitives_ros primitive_gamepad_launch.py
```


## Troubleshooting

* If one of the robots stop moving and you see a `libfranka: Move command aborted` error in terminal 1, take the following steps:
    1. Stop both programs in the terminals.
    2. Press and release both e-stops.
    3. Restart both programs in both terminals.

* If one the robots stops moving, you see the following error in terminal 1 `Shutting down control loop in PandaInterface due to libfranka: Move command aborted: motion aborted by reflex! ["joint_position_limits_violation"]` or `Shutting down control loop in PandaInterface due to libfranka: Move command aborted: motion aborted by reflex! ["self_collision_avoidance_violation"]`, and the prior steps do NOT work, do the following:
    1. Stop both programs in the terminals.
    2. Press both e-stops.
    3. Use the 2 parallel buttons on the panda end-effector to guide the prior stopped robot back into its home pose.
    4. Release both e-stops.
    5. Restart both programs in both terminals.