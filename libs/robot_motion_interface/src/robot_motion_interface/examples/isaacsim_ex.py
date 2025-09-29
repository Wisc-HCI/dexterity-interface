from robot_motion_interface.isaacsim.isaacsim_interface import IsaacsimInterface

import os
import time
import threading

import numpy as np



def oscillate_setpoint(isaac, base_setpoint, idxs, amplitude=0.3, period=2.0):
    """
    Continuously sinusoidally oscillates specified joint indices in a separate thread.

    Args:
        isaac (IsaacsimInterface): The simulation interface instance.
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
        isaac.set_joint_positions(setpoint)
        time.sleep(0.05)  # ~20Hz update

def example():
    """
    Simple example of bimanual arms oscillating
    """
    cur_dir = os.path.dirname(__file__)
    config_path = os.path.join(cur_dir, "..", "isaacsim", "config", "isaacsim_config.yaml")

    isaac = IsaacsimInterface.from_yaml(config_path)


    setpoint = np.zeros(38)
    setpoint[:14] = np.array([0.0, 0.0, -np.pi/4, -np.pi/4, 0.0, 0.0,
        -3*np.pi/4, -3*np.pi/4, 0.0, 0.0, np.pi/2, np.pi/2, np.pi/4, np.pi/4])
    isaac.set_joint_positions(setpoint)

    idxs = [4, 5, 6, 7, 30, 31, 32, 33, 34, 35, 36, 37]  
    
    thread = threading.Thread(target=oscillate_setpoint, args=(isaac, setpoint, idxs))
    thread.daemon = True
    thread.start()

    isaac.start_simulation()
    
if __name__ == "__main__":
   
    example()
