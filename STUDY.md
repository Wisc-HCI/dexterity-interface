# Study Setup

## ON DESKTOP (3 terminals):

1. Terminal 1
```bash
xhost +local: 
docker compose -f compose.isaac.yaml run --rm isaac-base
```

2. Terminal 2:
```bash
docker compose -f compose.isaac.yaml exec isaac-base bash
```

3. Terminal 3:
```bash
npm run build --prefix app/ui_frontend/
```

4. Terminal 1
```bash
cd /workspace/libs/robot_motion_interface/ros
colcon build --cmake-clean-cache --symlink-install

cd /workspace/libs/primitives/ros
colcon build --cmake-clean-cache --symlink-install
cd /workspace
```


```bash
source libs/robot_motion_interface/ros/install/setup.bash
source libs/primitives/ros/install/setup.bash

ros2 launch primitives_ros sim.launch.py isaac_args:='--kit_args=--/app/window/hideUi=true'
```
5. Terminal 2
```bash
source libs/robot_motion_interface/ros/install/setup.bash
source libs/primitives/ros/install/setup.bash
```

Run one of these:
```bash
# Task 1 and 2
uvicorn ui_backend.api:app

# Task 3
TASK=3 uvicorn ui_backend.api:app
```

6. Terminal 3
```bash

npm run dev --prefix app/ui_frontend/
```

7. Navigate to one of these:
```bash
# CONDITION 1 (no plan)
http://127.0.0.1:3000?show_plan=false

# CONDITION 2 (no plan editing/interaction)
http://127.0.0.1:3000?plan_interaction=false

# CONDITION 3 (full system)
http://127.0.0.1:3000
```


## ON LAPTOP (single terminal):

```bash
xhost +local:
docker compose -f compose.ros.yaml run --rm ros-base
```

```bash
cd /workspace/libs/robot_motion_interface/ros
colcon build --cmake-clean-cache --symlink-install

cd /workspace/libs/primitives/ros
colcon build --cmake-clean-cache --symlink-install
cd /workspace
```

```bash
source libs/robot_motion_interface/ros/install/setup.bash
source libs/primitives/ros/install/setup.bash

ros2 launch primitives_ros real.launch.py
```


## If camera needs calibration (run on Desktop in termain)
```bash
cd libs/sensor_interface/sensor_interface_py/src/sensor_interface/camera/config
python calibrate_T_world_color.py --config realsense_config.yaml --marker-size 0.1 --marker-pos 0 0 0.9369 --write
```