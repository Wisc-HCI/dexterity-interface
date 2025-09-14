# Dexterity Interface

## Requirements

* For the simulation/interface you will need:
    * Ubuntu Machine  with EITHER:
        * [Docker Engine](https://docs.docker.com/engine/install/). TODO: Add functionality.
        OR
        * Ubuntu 22.04

* For Running on the Panda you will additionally need:
    * Above requirements.
    * Ubuntu Machine with the following added:
        * The [Realtime Kernel Patch Kernel Patch](https://frankaemika.github.io/docs/installation_linux.html#setting-up-the-real-time-kernel).
    * Franka Emika Panda 7 DOF Robot setup with the [FCI](https://frankaemika.github.io/docs/getting_started.html).
        * Robot system version: 4.2.X (FER pandas)
        * Robot / Gripper Server version: 5 / 3
    * [Axio80-M20 Force Torque Sensor](https://www.ati-ia.com/products/ft/ft_models.aspx?id=Axia80-M20) installed on the Panda's End Effector.



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


## Setup
1. Install Ubuntu dependencies:
```bash
sudo apt update
sudo apt install libeigen3-dev
```


## Running


## System Architecture
```mermaid
flowchart TD

%% --- UI (top) ---
subgraph L1["UI"]
  UIF["UI Frontend (Javascript, ?? Framework)"]
  UIB["UI Backend (ROS Python, ?? Framework)"]
  UIF --- UIB
end

%% --- High-level logic / Primitives ---
subgraph L2["High-level Logic & Primitives"]
  PRIM["Primitives (ROS Python): both high and low"]
  PLAN["Planning (Python): Breakdown task into primitives"]
  LLM["LLM (Python):task breakdown/chat "]
  PERC["Perception (Python): object localization, manipulation points"]
end

%% --- Interfaces ---
subgraph L3["Hardware/Software Interfaces"]
  ROSI["ROS Motion Wrapper (ROS Python)"]
  RCTL["Robot Motion Interface (Python, C++): Panda, Tesollo, IsaacSim"]
  ISAACI["IsaacSim UI Interface (Python): add/remove objects, click/drag, extends Isaacsim Robot Motion interface"]
  SENSI["Sensor Interface (Python, C++): force torque sensor"]
end

%% --- Low level (bottom) ---
subgraph L4["Low-level Control & Models"]
  CTRLS["Controllers (C++): cartesian torque, joint torque, joint pos/vel"]
  IK["IK (C++): Drake or RelaxedIK"]
  RPROPS["Robot Properties (C++): friction, coriolis"]
end

%% --- Wiring (top → bottom) ---
UIB --- PRIM
UIB --- PLAN
UIB --- LLM
UIB --- PERC
UIB --- ISAACI

PRIM --- ROSI
LLM --- PLAN

ROSI --- RCTL
PERC --- SENSI


RCTL --- CTRLS
CTRLS --- IK
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