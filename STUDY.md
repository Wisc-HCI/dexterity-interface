# Study Setup
## ON DESKTOP:

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

6. Terminal 3
```bash

npm run dev --prefix app/ui_frontend/
```


