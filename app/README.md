# Dexterity Interface Application

TODO: figure out if want to use this or ROS web bridge (may be better for recording data, handling streams, etc.)

## Dependencies
<!-- ```bash
pip install -e libs/sensor_interface/sensor_interface_py
pip install -e libs/planning/planning_py

pip install numpy==1.26 
pip install ulid-py 
``` -->

Make sure you are in the docker container (see root readme.md)
```bash
cd /workspace/libs/robot_motion_interface/ros
colcon build --symlink-install

cd /workspace/libs/primitives/ros
colcon build --symlink-install
cd /workspace
```

## Setup
Recommend doing this in a virtual environment:

```bash
pip3 install -e app/ui_backend
npm install --prefix app/ui_frontend
```
## Running
<!-- Make sure you are in root folder (`dexterity-interface`) and have sourced `venv-dex` before running these in seperate terminals:

Start the Isaacsim UI in streaming mode:
```bash
LIVESTREAM=2  python3 -m robot_motion_interface.isaacsim.isaacsim_interface --kit_args="--/app/window/hideUi=true --/app/window/drawMouse=false"
``` -->

TODO: MORE SETUP INSTRUCTIONS
<!-- Make sure these 2 nodes are running:
```bash
# 1 Run this in one terminal
source libs/robot_motion_interface/ros/install/setup.bash
# TODO: ADD --kit_args="--/app/window/hideUi=true --/app/window/drawMouse=false"
LIVESTREAM=2  ros2 run robot_motion_interface_ros interface --ros-args -p interface_type:=isaacsim_object -p config_path:=/workspace/libs/robot_motion_interface/config/isaacsim_config.yaml

# Run this in another terminal
source libs/primitives/ros/install/setup.bash
ros2 launch primitives_ros twin.launch.py
``` -->

```bash
source libs/robot_motion_interface/ros/install/setup.bash
source libs/primitives/ros/install/setup.bash

# Launch simulation on computer with GPU:
ros2 launch primitives_ros sim.launch.py

# [OPTIONAL] If you also want the real interface, launch this on the computer with the kernel patch:
ros2 launch primitives_ros real.launch.py
```
Start API server (this must be done in terminal where ros is sourced (i.e. in one of the docker containers)):
```bash
source libs/primitives/ros/install/setup.bash
uvicorn ui_backend.api:app --reload
```

Start frontend server:
```bash
npm run dev --prefix app/ui_frontend/
```

NOTE: If you instead want to build and run the frontend for production, run the following:
```bash
npm run build --prefix app/ui_frontend/
npx serve app/ui_frontend/dist
```


In your web browser, go to the following to test:
* API: http://127.0.0.1:8000/api/test
* API docs: http://127.0.0.1:8000/docs
* Front end: http://127.0.0.1:3000
