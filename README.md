# Dexterity Interface

## Get LLM's instructions
1. First, you need to create a .env file in this folder with the OpenAI credentials. It should be in this format:
    ```bash
    OPENAI_API=YOUR_API_KEY_HERE
    ```

2. Next, start a python virtual environment in this directory:

    Linux/Mac:
    ```bash
    python3 -m venv venv
    source venv/bin/activate  
    ```

    Windows:
    ```powershell
    python3 -m venv venv
    .\venv\Scripts\Activate.ps1
    ```

    If this windows command doesn't work, you may have to run this in an Admin shell first:
    ```powershell
    set-executionpolicy remotesigned
    ```

3. Next, install all the python requirements:
    ```bash
    pip install -r requirements.txt
    ```
4. Then, you can run the llm script with:
    ```bash
    python3 chat.py
    ```
    Note: if your venv has become deactivated, you may need to reactivate it with the activate command in the Setup section.

## Set up the repository and run the container
1. Check prerequisites for [panda-primitives](https://github.com/Wisc-HCI/panda-primitives) and [panda-primitives-control](https://github.com/Wisc-HCI/panda-primitives-control)
   
2. Bring in the submodules:
    ```bash
    git submodule update --init --recursive
    ```
3. Setup display forwarding:
    ```bash
    xhost +local:
    ```
4. Now  build the container image and start the container. Make sure you are in this root directory. These commands mount on the current directory as the containers file system so any changes you make to the files on your host machine will be mirrored in the container. These commands also allow the containers display to be forwarded to your host machine so that you can see it.
    ```bash
    sudo docker build -t llm-control .

    sudo docker run --rm -it --privileged --cap-add=SYS_NICE --device=/dev/input/event* --env DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v $(pwd):/workspace --net=host llm-control
    ```

## Compile panda-primitives package
1. Run:
    ```bash
    cd panda-primitives/
    catkin build authoring
    cd ..
    ```	
2. Then, run:
    ```bash
    source panda-primitives/devel/setup.bash
    ```
    
## Compile panda-primitives-control package(SKIP FOR SIMULATION)
This step will compile panda-primitives-control package that control Panda Robot in low level, you can skip it if you only need simulation

1. Add necessary environment variables: Replace with your Panda's IP
    ```bash
    export PANDA_IP=192.168.1.3
    ```
2. ### Compile non-ROS package (PandaController)

    If first time, first configure:
    ```bash
    cd panda-primitives-control/src/PandaController
    mkdir build
    cd build
    cmake ..
    cd ../../../..
    ```

    Anytime, run:
    ```bash
    cd panda-primitives-control/src/PandaController/build
    make install
    cd ../../../..
    ```

    ### Compile ROS package
    Compile individually each ros packages:
    ```bash
    cd panda-primitives-control
    # catkin build relaxed_ik --no-notify
    catkin build panda_ros_msgs --no-notify
    catkin build panda_ros --no-notify
    #catkin build dmp_deformations --no-notify
    catkin build inputs_ros --no-notify
    catkin build controller --no-notify
    cd ..
    ```
3. Run with ROS
    1. Use Franka Desktop to unlock the Panda's joints and enable FCI mode.
    2. Run `source panda-primitives-control/devel/setup.bash` inside the root directory
    3. Start the launch files related to the application:
		```bash
		roslaunch controller mover_test.launch
		```
For more information, please refer to [panda-primitives-control](https://github.com/Wisc-HCI/panda-primitives-control)

## Run

1. In a seperate terminal(second, still in your docker container), Run one of the following:
    * [SIMULATION] `roslaunch authoring all.launch only_virtual:=true`
    * [ON ROBOT] `roslaunch authoring all.launch`

2. Once we finished talking with llm, its response will be record at output.txt, then in a seperate terminal (third, still in your docker container), run:
    ```bash
    roslaunch authoring llm_control.launch
    ```
    You can also run any other scripts in the authoring/test folder in this same manner.

---
