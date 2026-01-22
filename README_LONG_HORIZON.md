# Notes for Long Horizon Agent Study

## Configuration
To add back high-level primitives, uncomment them in: libs/planning/planning_py/src/planning/llm/config/primitives.yaml

To adjust the scene, go to: app/ui_backend/src/ui_backend/utils/helpers.py

To change the model, adjust OPENAI_MODEL in: libs/planning/.env

Executed plans are stored in: app/ui_backend/src/ui_backend/json_save

## Setup
This assumes everything has already been compiled (see other readmes).

Terminal 1 (Simulator):
```bash
cd ~/Desktop/github/HCI/dexterity-interface/
docker compose -f compose.isaac.yaml run --rm isaac-base

source libs/robot_motion_interface/ros/install/setup.bash
source libs/primitives/ros/install/setup.bash
```

Terminal 2 (Backend):
```bash
cd ~/Desktop/github/HCI/dexterity-interface/
docker compose -f compose.isaac.yaml exec isaac-base bash

source libs/robot_motion_interface/ros/install/setup.bash
source libs/primitives/ros/install/setup.bash
```


Terminal 3 (Frontend):
```bash
cd ~/Desktop/github/HCI/dexterity-interface/
```


## Running
Terminal 1:
```bash
ros2 launch primitives_ros sim.launch.py
```

Terminal 2:
```bash
uvicorn ui_backend.api:app --reload
```

Terminal 3:
```bash
npm run dev --prefix app/ui_frontend/
```