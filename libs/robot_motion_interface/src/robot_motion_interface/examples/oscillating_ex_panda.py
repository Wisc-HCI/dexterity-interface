# TODO: Merge this example with isaacsim one
from robot_motion_interface.interface import Interface
from robot_motion_interface.panda.panda_interface import PandaInterface

import os
import time
import threading

import numpy as np



def oscillate_setpoint(interface: Interface, base_setpoint:np.ndarray, idxs:list[int], 
                       amplitude:float=0.3, period:float=2.0):
    """
    Continuously sinusoidally oscillates specified joint indices in a separate thread.

    Args:
        isaac (Interface): The interface instance (can be IsaacsimInterface, PandaInterface, etc)
        base_setpoint (np.ndarray): (n_idxs) The base joint positions to oscillate around.
        idxs (list[int]): (n_idxs) Indices of joints to oscillate.
        amplitude (float): Amplitude of oscillation (radians). Default is 0.3.
        period (float): Period of oscillation in seconds. Default is 2.0.
    """
    while True:
        t = time.time()
        setpoint = base_setpoint.copy()
        # Oscillate selected joints
        for i in idxs:
            setpoint[i] += amplitude * np.sin(2 * np.pi * t / period)

        # print("SETPOINT:", setpoint)
        interface.set_joint_positions(setpoint)
        time.sleep(0.05)  # ~20Hz update

def main():
    """
    Simple example of bimanual arms oscillating
    """
    cur_dir = os.path.dirname(__file__)
    config_path = os.path.join(cur_dir, "..", "panda", "config", "right_panda_config.yaml")

    interface = PandaInterface.from_yaml(config_path)


    setpoint = np.zeros(38)
    setpoint = np.array([0.0, -np.pi/4,  0.0, -3*np.pi/4, 0.0,  np.pi/2, np.pi/4]) # home
    interface.set_joint_positions(setpoint)

    idxs = [3, 4, 5]  


    
    osc_thread = threading.Thread(target=oscillate_setpoint, args=(interface, setpoint, idxs))
    osc_thread.start()

    print("STARTED OSC THREAD")

    # Panda Control loop needs to be in its own thread since its extremely resource heavy
    control_thread = threading.Thread(target=interface.start_loop)
    control_thread.start()

    print("STARTED CONTROL THREAD")

    # Keep the main thread alive
    while True:
        time.sleep(1)


if __name__ == "__main__":
   
    main()
