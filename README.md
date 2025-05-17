# Dexterity Interface

## Requirements

* For the simulation/interface you will need:
    * Ubuntu Machine A with:
        * [Docker Engine](https://docs.docker.com/engine/install/).
        * A live server. For VScode, we recommend [Live Server Extension](https://marketplace.visualstudio.com/items?itemName=ritwickdey.LiveServer).

* For Running on the Panda you will additionally need:
    * Above requirements.
    * Ubuntu Machine A with the following added:
        * The [Realtime Kernel Patch Kernel Patch](https://frankaemika.github.io/docs/installation_linux.html#setting-up-the-real-time-kernel).
    * Franka Emika Panda 7 DOF Robot setup with the [FCI](https://frankaemika.github.io/docs/getting_started.html).
        * Robot system version: 4.2.X (FER pandas)
        * Robot / Gripper Server version: 5 / 3
    * [Axio80-M20 Force Torque Sensor](https://www.ati-ia.com/products/ft/ft_models.aspx?id=Axia80-M20) installed on the Panda's End Effector.

* For Running with the Vision and Kinect you will additionally need:
    * Ubuntu Machine B with: 
        * Nvidia GPU.
        * [Docker Engine](https://docs.docker.com/engine/install/).
    * Azure Kinect


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

    %% Force Torque Sensor
    subgraph FTS["**Force Torque Sensor**"]
        FTS_IP["IP: 192.168.2.2 <br> Netmask: 255.255.255.0"]:::ethernet
        FTS_PWR["Barrel Jack"]:::power_data
    end

    %% FT Power Supply
    subgraph FTPS["**Force Torque Sensor Power Supply**"]
        FT_DESC["Must be powered on."]:::description
    end

    %% Kinect
    subgraph KN["**Azure Kinect Camera**"]
        KN_PWR["Barrel Jack"]:::power_data
        KN_USBC["USB C"]:::power_data
    end

    %% Machine A
    subgraph MA["**Machine A**"]
        MA_DESC["Requires Real-Time Kernel Patch. Runs Panda controller (backend.launch). "]:::description
        MA_IP1["IP: 192.168.1.5 <br> Netmask: 255.255.255.0"]:::ethernet
        MA_IP2["IP: 192.168.2.5 <br> Netmask: 255.255.255.0"]:::ethernet
        MA_IP3["IP: 192.168.3.2 <br> Netmask: 255.255.255.0"]:::ethernet
    end

    %% Machine B
    subgraph MB["**Machine B**"]
        MB_DESC["Requires Nvidia GPU to run VLM (vision.launch). "]:::description
        MB_IP["IP: 192.168.3.3 <br> Netmask: 255.255.255.0"]:::ethernet
        MB_USBA["USB A"]:::power_data
        MB_USBC["USB C"]:::power_data
    end

    %% Wall Outlets
    PC_PWR["Wall Outlet"]:::power_data
    FTPS_PWR["Wall Outlet"]:::power_data
    MB_PWR["Wall Outlet"]:::power_data

    %% Connections
    PC_PWR --- PC_CONN --- PA_CONN --- PA
    FTS_PWR --- FTPS --- FTPS_PWR
    MB --- MB_PWR
    
    KN_PWR --- MB_USBA
    KN_USBC --- MB_USBC
    

    PA_IP --- MA_IP1
    FTS_IP --- MA_IP2
    MA_IP3 --- MB_IP

    %% Styles
    classDef description fill:none,stroke:none,color:#000;
    classDef ethernet fill:#fff3b0,stroke:#000,color:#000;
    classDef power_data fill:#f5b7b1,stroke:#000,color:#000;
```


## Setup and Run on Machine A
Make sure you are running the following on Machine A.

### 1. Setup LLM configs
1. First, you need to create a .env file in this directory with the OpenAI credentials. It should be in this format:
    ```bash
    OPENAI_API=YOUR_API_KEY_HERE
    ```

### 2. Set up the repository and run the container
1. Bring in the submodules and/or make sure they are updated:
    ```bash
    git submodule update --init --recursive
    ```

2. Now  build the container image and start the container. Make sure you are in this root directory and in a bash terminal. These commands mount on the current directory as the containers file system so any changes you make to the files on your host machine will be mirrored in the container. These commands also allow the containers display to be forwarded to your host machine so that you can see it.

    ```bash
    sudo docker build -t dex-interface .
    ```

    IF you are only running in simulation, run:
    ```bash
    sudo docker run --rm -it --privileged --cap-add=SYS_NICE --device=/dev/input/event* --env DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v $(pwd):/workspace --net=host dex-interface
    ```

    ELSE if you are running on the Panda, run:
    ```bash
    sudo docker run --rm -it --privileged --cap-add=SYS_NICE --device=/dev/input/event* --env DISPLAY=$DISPLAY --env ROS_MASTER_URI=http://192.168.3.2:11311 --env ROS_IP=192.168.3.2 --env PANDA_IP=192.168.1.3 -v /tmp/.X11-unix:/tmp/.X11-unix -v $(pwd):/workspace --net=host dex-interface
    ```


### 2.5. [SKIP FOR JUST SIMULATION] Setup the Robot
Use Franka Desktop (at 192.168.1.3) to unlock the Panda's joints and enable FCI mode.


### 3. Compile  packages
1. Compile C++ package (PandaController)
    ```bash
    cd backend-ros/src/PandaController/
    mkdir -p build
    cd build
    [ -f Makefile ] || cmake ..  # Only initialize if not  already
    make install
    cd ../../../..
    ```

2. Compile Ros Packages
    ```bash
    cd backend-ros
    catkin build
    source devel/setup.bash
    ```

### 4. Running
1. Launch the Backend. Make sure you are in the `backend-ros/` directory when running these commands.

    a. IF you want the program to run on the robot, run each of these in their own terminal (see the [Multiple Docker Terminals Section](#multiple-docker-terminals) for instructions on how to do this).
    ```bash
    roscore
    ```

    ```bash
    roslaunch interface backend.launch only_virtual:=false
    ```

    ELSE if you just want to run in simulation,  run:
    ```bash
    roslaunch interface backend.launch only_virtual:=true 
    ```

    b. [SKIP IF USING Kinect on Machine B] For a simulation of 3 random moving objects in the scene, run this in another terminal:
    ```bash
    roslaunch interface vision.launch use_kinect:=false
    ```

2. Launch a live server for `frontend/index.html`. If you are using VScode, you can do that by selecting that file to open it, and in the lower right of VSCode click "Go Live". This should launch the interface in your browser.



## [SKIP IF NOT USING Kinect] Setup and Running on Machine B
Make sure you are running the following on Machine B.
1. Build the container image and start the container. Make sure you are in this root directory and in the bash terminal.
    ```bash
    xhost +local:

    sudo docker run -it --rm --gpus all --privileged -e DISPLAY=$DISPLAY -e PULSE_SERVER=unix:/run/user/1000/pulse/native --env ROS_MASTER_URI=http://192.168.3.2:11311 --env ROS_IP=192.168.3.3 -v /run/user/1000/pulse:/run/user/1000/pulse -v /tmp/.X11-unix:/tmp/.X11-unix -v $(pwd):/workspace --device /dev/snd --device /dev/bus/usb --net=host dex-interface  
    ```

2. Compile ros packages
    ```bash
    cd backend-ros
    catkin build interface
    source devel/setup.bash
    ```

## Running

```bash
roslaunch interface vision.launch use_kinect:=true
```


## Documentation

### ROS Packages and Topics
This is a non-comprehensive diagram of the most important files, packages, and topics in this system.
```mermaid
flowchart TB
    %% Legend / Key
    subgraph Key["Legend"]
        k1["File"]:::file
        k2(["Topic"]):::topic
        k3(["Service"]):::service

    end

    %% Frontend
    subgraph frontend
        LLMChat["**LLMChat.js** <br> Handles chat interface"]:::file
        Visualizer["**Visualizer.js** <br> Handles robot and object visualization"]:::file
    end

    %% interactive_marker_proxy
    subgraph interactive_marker_proxy
        proxy["**src/proxy.cpp** <br> <u>Node:</u> proxy <br> Middleman for handling objects in scene to display"]:::file
    end

    %% Interface
    subgraph interface
        llm_handler["**scripts/llm_handler.py** <br> <u>Node:</u> llm_handler <br> Sends/receives messages from ChatGPT"]:::file
        scene_handler["**scripts/scene_handler** <br> <u>Node:</u> scene_handler <br> Keeps track of objects in scene"]:::file
        vision_interface["**scripts/vision_interface** <br> <u>Node:</u> vision_interface <br> Pings Kinect ~1s to detect objects"]:::file
        llm_executor["**scripts/llm_executor.py** <br> <u>Node:</u> panda_command_executor <br> Parses ChatGPT response into primitives"]:::file
    end

    %% panda_primitives
    subgraph panda_primitives
        planner["**nodes/planner** <br> <u>Node:</u> planner <br> Breaks high level primitives into lower level ones"]:::file
        mover_server["**nodes/mover_server** <br> <u>Node:</u> mover_server <br> Logic for lower level primitives"]:::file
    end

    %% panda_ros
    subgraph panda_ros
        panda_node["**src/panda_ros.cpp** <br> <u>Node:</u> panda_controller <br> Executes lowest level code on robot"]:::file
    end

    %% Topics and Services
    user_response(["/user_response"]):::topic
    llm_response(["/llm_response"]):::topic
    llm_commands(["/llm_commands"]):::topic
    parser_command(["/parser/command"]):::service
    scene_object_ctrl(["/scene/object_controls/update"]):::topic
    scene_object_ctrl_tunnel(["/scene/object_controls/tunneled/update"]):::topic
    scene_objects(["/scene/objects"]):::topic
    scene_vision(["/scene/vision/objects"]):::topic
    mover_srv(["/mover_server"]):::topic
    panda_ctrl(["/panda/control_wrench"]):::topic
    panda_joint(["/panda/joint_states"]):::topic
    panda_cart(["/panda/cart_velocity"]):::topic
    panda_cmd(["/panda/commands"]):::topic
    panda_pose(["/panda/hybrid_pose"]):::topic

    %% Frontend links
    LLMChat -- Publisher --> user_response
    llm_response -- Subscriber --> LLMChat
    scene_object_ctrl_tunnel -- Subscriber --> Visualizer


    %% Interactive Marker Proxy links
    scene_object_ctrl -- Subscriber --> proxy 
    proxy -- Publisher --> scene_object_ctrl_tunnel



    %% Interface links
    user_response -- Subscriber --> llm_handler
    llm_handler -- Publisher --> llm_commands
    llm_handler  -- Publisher --> llm_response
    llm_handler -- Subscriber --> scene_objects
    scene_objects -- Publisher --> scene_handler
    scene_handler -- Publisher --> scene_object_ctrl
    scene_handler -- Subscriber --> scene_vision
    scene_vision -- Publisher --> vision_interface
    vision_interface -- Subscriber --> scene_vision
    llm_commands -- Subscriber --> llm_executor
    llm_executor -- Publisher --> parser_command

    %% panda_primitives
    parser_command -- Subscriber --> planner
    planner -- Publisher --> mover_srv
    mover_srv -- Subscriber --> mover_server
    
    mover_server -- Publisher --> panda_cmd
    mover_server -- Publisher --> panda_pose
    panda_ctrl -- Subscriber --> mover_server
    panda_joint -- Subscriber --> mover_server
    panda_cart -- Subscriber --> mover_server


    %% panda_ros
    panda_cmd -- Subscriber --> panda_node
    panda_pose -- Subscriber --> panda_node
    panda_node -- Publisher --> panda_ctrl
    panda_node -- Publisher --> panda_joint
    panda_node -- Publisher --> panda_cart

    %% Styling
    classDef file fill:#fff3b0, stroke:#000, color:#000;
    classDef topic fill:#d6b3ff, stroke:#000,  color:#000;
    classDef service fill:#ffb3d1,stroke:#000, color:#000;

```


### Multiple Docker Terminals
To open multiple another terminal to you docker container, first run the following on your local machine:
```bash
sudo docker ps
```
This will give you a list of containers with container IDs. Then run the following with your container ID:
```bash
sudo docker exec -it YOUR_CONTAINER_ID bash
```
Now run the following in your newly opened terminal to source the workspace.
```bash
cd backend-ros
source devel/setup.bash
```  


### Troubleshooting
Here are some commands that are nice to use for troubleshooting
```bash
# To view submodule status
git submodule status


# To view TF frames
rosrun tf2_tools view_frames.py
```


### Submodules
These are the submodules used in this project
- [authoring_msgs](https://github.com/emmanuel-senft/authoring-msgs)
- [interactive_marker_proxy_noetic](https://github.com/schromya/interactive_marker_proxy_noetic/tree/tf-lifetime-fix): Make sure you are on the tf-lifetime-fix branch
- [panda_ros](https://github.com/Wisc-HCI/panda_ros)
- [panda-primitives](https://github.com/Wisc-HCI/panda-primitives)
- [panda-ros-msgs](https://github.com/emmanuel-senft/panda-ros-msgs/tree/study): Make sure you are on the study branch
- [PandController](https://github.com/Wisc-HCI/PandaController)
- [ros_numpy](https://github.com/eric-wieser/ros_numpy)
- [rviz_camera_stream](https://github.com/lucasw/rviz_camera_stream)
- [assistive_robotics_thesis](https://github.com/rusmilov/assistive_robotics_thesis/tree/hand-integration): Make sure you are on the hand-integration branch


### Resources
* https://robotwebtools.github.io/
* https://github.com/Mechazo11/interactive_marker_proxy_noetic
* https://github.com/ros-visualization/visualization_tutorials/tree/noetic-devel/interactive_marker_tutorials
* http://wiki.ros.org/roslibjs/Tutorials/BasicRosFunctionality
* https://robotwebtools.github.io/ros3djs/
* https://docs.ros.org/en/noetic/api/visualization_msgs/html/msg/InteractiveMarkerControl.html


