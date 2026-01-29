# Dexterity Interface Application


## Dependencies
* CONTAINER 1: compose.isaac.yaml docker container (instructions at root of repo) run on machine with Nvidia GPU (only tested on Ubuntu 22.04 or 24.04)
* npm installed on machine running the above
* CONTAINER 2: [OPTIONAL for running on robot] compose.ros.yaml docker container (instructions at root of repo) run on machine with Kernal patch (only tested on Ubuntu 22.04 or 24.04)
*  [OPTIONAL for running on robot] robot hardware specified in the root readme.

## Setup
In CONTAINER 1, run:
```bash
cd /workspace/libs/robot_motion_interface/ros
colcon build --symlink-install

cd /workspace/libs/primitives/ros
colcon build --symlink-install
cd /workspace
```

On the same machine that CONTAINER 1 is running, run:
```bash
npm install --prefix app/ui_frontend
```

[OPTIONAL] In CONTAINER 2, run:
```bash
cd /workspace/libs/robot_motion_interface/ros
colcon build --symlink-install

cd /workspace/libs/primitives/ros
colcon build --symlink-install
cd /workspace
```

## Running

1. In CONTAINER 1, run:
    ```bash
    source libs/robot_motion_interface/ros/install/setup.bash
    source libs/primitives/ros/install/setup.bash

    # Launch simulation on computer with GPU:
    ros2 launch primitives_ros sim.launch.py
    ```

2. In another terminal in CONTAINER 1, run:
    ```bash
    source libs/robot_motion_interface/ros/install/setup.bash
    source libs/primitives/ros/install/setup.bash
    uvicorn ui_backend.api:app --reload
    ```

3. WAIT until  the terminal for STEP 1 says "Creating window for environment". Then in a terminal on the same machine as CONTAINER 1, run:
    ```bash
    npm run dev --prefix app/ui_frontend/
    ```

    NOTE: If you instead want to build and run the frontend for production, run the following:
    ```bash
    npm run build --prefix app/ui_frontend/
    npx serve app/ui_frontend/dist
    ```

4. [OPTIONAL for running on robot] In CONTAINER 2, run:
    ```bash
    source libs/robot_motion_interface/ros/install/setup.bash
    source libs/primitives/ros/install/setup.bash
    ros2 launch primitives_ros real.launch.py
    ```

5. In your web browser, go to the following to test:
* Front end: http://127.0.0.1:3000
* API docs: http://127.0.0.1:8000/docs
* API: http://127.0.0.1:8000/api/<PATH_HERE>