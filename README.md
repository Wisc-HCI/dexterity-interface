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


## Setup and Run


