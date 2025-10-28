# Dexterity Interface

## Requirements
* For the simulation/interface you will need:
    * Ubuntu Machine (22.04 or 24.04 recommended).
* You can also have the following hardware requirements:
    * Franka Emika Panda 7 DOF Robot setup with the [FCI](https://frankaemika.github.io/docs/getting_started.html).
        * Robot system version: 4.2.X (FER pandas)
        * Robot / Gripper Server version: 5 / 3
        * The [Realtime Kernel Patch Kernel Patch](https://frankaemika.github.io/docs/installation_linux.html#setting-up-the-real-time-kernel) on Ubuntu Machine.
    * [Tesollo 3 Finger Gripper]() TODO
    * Cameras TODO




<br>

If you are running on the robot, be sure to set the proper static IPs and wiring that you need as shown below:

```mermaid
flowchart LR

    %% Panda Controller
    subgraph PC["**Panda Controller**"]
        PC_CONN["Round Pin Connector"]:::power_data
    end

    %% Panda Arm
    subgraph PA["**Panda Robot**"]
        PA_IP["IP: 192.168.1.3 <br> Netmask: 255.255.255.0"]:::ethernet
        PA_CONN["Round Pin Connector"]:::power_data
    end



    %% Ubuntu Machine
    subgraph MA["**Ubuntu Machine**"]
        MA_DESC["Requires Real-Time Kernel Patch if running robot."]:::description
        MA_IP1["IP: 192.168.1.5 <br> Netmask: 255.255.255.0"]:::ethernet
        MA_IP2["IP: 192.168.2.5 <br> Netmask: 255.255.255.0"]:::ethernet
        MA_IP3["IP: 192.168.3.2 <br> Netmask: 255.255.255.0"]:::ethernet
    end



    %% Wall Outlets
    PC_PWR["Wall Outlet"]:::power_data

    %% Connections
    PC_PWR --- PC_CONN --- PA_CONN --- PA
    

    PA_IP --- MA_IP1

    %% Styles
    classDef description fill:none,stroke:none,color:#000;
    classDef ethernet fill:#fff3b0,stroke:#000,color:#000;
    classDef power_data fill:#f5b7b1,stroke:#000,color:#000;
```

## Option 1: Docker Setup
This allows you to run isaacsim with docker. These instructions are an adapted version of [these](https://docs.isaacsim.omniverse.nvidia.com/5.0.0/installation/install_container.html) and [these](https://isaac-sim.github.io/IsaacLab/main/source/deployment/docker.html)

1. Install Docker by following the `Install using the apt repository` instruction [here](https://docs.docker.com/engine/install/ubuntu/#install-using-the-repository).

2. Install Nvidia Container Toolkit by following [these instructions](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html). We recommend version 1.17.8 but other versions may work (although we know for sure that version 1.12 has Vulkan issues). 
    * Make sure you complete the `Installation` section for `With apt: Ubuntu, Debian` and also the `Configuring Docker` section.
    * To check proper installation, please run `sudo docker run --rm --runtime=nvidia --gpus all ubuntu nvidia-smi`. This should output a table with your Nvidia driver. If you run into `Failed to initialize NVML: Unknown Error`, reference [this post](https://stackoverflow.com/questions/72932940/failed-to-initialize-nvml-unknown-error-in-docker-after-few-hours) for the solution.

3. Install Docker compose by following there `Install using the repository` [instructions here](https://docs.docker.com/compose/install/linux/#install-using-the-repository).

4. Run the following to build and launch the docker container. This will take a while the first time you run them:

    ```bash
    xhost +local: # Note: This isn't very secure but is th easiest way to do this
    docker compose -f compose.isaac.yaml build
    docker compose -f compose.isaac.yaml run --rm isaac-base
    
    ```

    > NOTE: if you need to start another terminal, once the container is started, run `sudo docker compose -f compose.isaac.yaml exec isaac-base bash`
5. Start Isaacsim in the container terminal by running one of the following:
    ```bash
    
    /isaac-sim/runheadless.sh  # NO GUI
    /isaac-sim/isaac-sim.sh   # GUI
    ```


## Option 2: Python Setup
You will only be able to run the python packages and IsaacSim, NOT any of the ROS packages.
1. Install Ubuntu dependencies:
    ```bash
    sudo apt update
    sudo apt install libeigen3-dev python3.11 python3.11-venv
    ```

2. Create and source python virtual environment
    ```bash
    python3.11 -m venv venv-dex
    source venv-dex/bin/activate
    ```

3. Install IsaacSim and IsaacLab. 
    ```bash
    pip install --upgrade pip
    pip install torch==2.7.0 torchvision==0.22.0 --index-url https://download.pytorch.org/whl/cu128
    pip install isaaclab[isaacsim,all]==2.2.0 --extra-index-url https://pypi.nvidia.com
    ```

    Test that Isaacsim installed correctly. The first time this is run, make sure to reply YES to the EULA prompt. Also the first time it may take a while to start as it sets up the necessary dependencies.
    ```bash
    isaacsim

    ```

4. Install our Python packages:
    ```bash
    pip install -e libs/robot_motion
    pip install -e libs/robot_motion_interface
    pip install -e libs/isaacsim_ui_interface/
    pip install -e libs/sensor_interface/sensor_interface_py

    pip install -e libs/primitives/primitives_py

    pip install -e libs/planning/planning_py
    ```
    
    TODO: Edit the rest of these so they are not so deep 

## Python Running
    ```bash
    python3 -m robot_motion.ik.ranged_ik

    python3 -m robot_motion_interface.isaacsim.isaacsim_interface
    python3 -m robot_motion_interface.tesollo.tesollo_interface
    python3 -m robot_motion_interface.panda.panda_interface
    python3 -m robot_motion_interface.bimanual_interface
    
    python3 -m isaacsim_ui_interface.isaacsim_ui_interface

    python3 -m sensor_interface.camera.kinect_interface
    python3 -m sensor_interface.camera.realsense_interface

    python3 -m primitives.primitive

    python3 -m planning.llm.gpt
    python3 -m planning.perception.yolo_perception
    ```


## TODO: C++ setup/running


## [ALTERNATIVE SETUP] Docker Setup
```bash
sudo docker build -t dex-interface .
sudo docker run --rm -it --privileged  -v $(pwd)/libs:/workspace/libs -v $(pwd)/app:/workspace/app --net=host dex-interface
```
Note: `--privileged` is not the safest, but it is an easy way to give real-time privileges to the container. TODO: Look into safer way.

## System Architecture
```mermaid
flowchart TD

%% --- UI (top) ---
subgraph L1["UI"]
  UIF["UI Frontend (Javascript, HTML, Tailwind CSS)"]
  UIB["UI Backend (Python, ROS Python, Fast API framework)"]
  UIF --- UIB
end

%% --- High-level logic / Primitives ---
subgraph L2["High-level Logic & Primitives"]
  PRIM["Primitives (Python): low and mid-level"]
  ROS_PRIM["ROS Primitive Wrapper (ROS Python)"]
  PLAN["Planning (Python): Breakdown task into primitives"]
  LLM["LLM (Python): task breakdown/chat "]
  PERC["Perception (Python): object localization, manipulation points"]
end

%% --- Interfaces ---
subgraph L3["Hardware/Software Interfaces"]
  ISAACI["IsaacSim UI Interface (Python): add/remove objects, click/drag, extends Isaacsim Robot Motion interface"]
  ROS_RMI["ROS Motion Interface Wrapper (ROS Python)"]
  RMI["Robot Motion Interface (Python, C++): Panda, Tesollo, IsaacSim"]

  SENSI["Sensor Interface (Python, C++): force torque sensor"]
  ROS_SENSI["ROS Sensor Interface Wrapper (Python, C++): cameras, force torque sensor"]
end

%% --- Low level (bottom) ---
subgraph L4["Low-level Control & Models"]
  CTRLS["Controllers (C++): cartesian torque, joint torque, joint pos/vel"]
  IK["IK (Python): Drake or RelaxedIK"]
  RPROPS["Robot Properties (C++): friction, coriolis"]
end

%% --- Wiring (top → bottom) ---
UIB --- ISAACI
UIB --- PLAN
UIB -.- PERC



PLAN --- ROS_PRIM
ROS_PRIM --- PRIM
PLAN --- LLM
PLAN --- PERC
PRIM --- ROS_RMI


RMI --- ROS_RMI
PERC --- SENSI
SENSI --- ROS_SENSI


RMI --- CTRLS
RMI --- IK
CTRLS --- RPROPS


```


## Mya notes:
* Isaac Sim 5.0 requires Python3.11, Ubuntu 22.04 or 24.04
* Ubuntu 22.04 is most compatible with ROS 2 Humble which by default uses Python3.10. So would need to compile from source for Python 3.11: https://github.com/isaac-sim/IsaacSim-ros_workspaces/blob/main/build_ros.sh
* Could instead update everything to Ubuntu 24.04 and use Jazzy.

### Todo:
* Figure out which packages are run on what computers.
* Figure out blocking vs non-blocking movement execution
* Allow partial setpoint updates.


https://docs.omniverse.nvidia.com/extensions/latest/ext_livestream/webrtc.html
https://isaac-sim.github.io/IsaacLab/main/source/api/lab/isaaclab.app.html


https://github.com/NVIDIA-Omniverse/web-viewer-sample

http://127.0.0.1:8211/streaming/webrtc-client?server=127.0.0.1
http://192.168.1.209:8211/streaming/webrtc-client?server=192.168.1.209




## Mya Notes (TODO: Move to Docker instructions)
xhost +local: # Note: This isn't very secure but is th easiest way to do this
docker compose -f compose.isaacv2.yaml build
docker compose -f compose.isaacv2.yaml run --rm isaac-base

### Install Conda
wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh
bash Miniconda3-latest-Linux-x86_64.sh
source ~/.bashrc

### https://isaac-sim.github.io/IsaacLab/main/source/setup/installation/source_installation.html

cd /isaaclab
./isaaclab.sh --conda 
conda activate env_isaaclab
./isaaclab.sh -i


pip install -e /workspace/libs/robot_motion
pip install -e /workspace/libs/robot_motion_interface

pip install colcon-common-extensions

source /humble_ws/install/setup.sh
cd /workspace/libs/robot_motion_interface/ros

colcon build --symlink-install \
  --cmake-args \
  -DPython3_EXECUTABLE=$(which python) \
  -DPYTHON_EXECUTABLE=$(which python)

source install/setup.bash

export PYTHONPATH=$PYTHONPATH:/root/miniconda3/envs/env_isaaclab/lib/python3.11/site-packages

ros2 run robot_motion_interface_ros interface --ros-args -p interface_type:=isaacsim -p config_path:=/workspace/libs/robot_motion_interface/config/isaacsim_config_docker.yaml

