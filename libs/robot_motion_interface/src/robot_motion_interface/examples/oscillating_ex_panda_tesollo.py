"""
Oscillates robot joints using Panda or IsaacSim interface.

Usage:
    TODO
"""

from robot_motion_interface.interface import Interface
from robot_motion_interface.panda.panda_interface import PandaInterface
from robot_motion_interface.tesollo.tesollo_interface import TesolloInterface


import os
import time
import threading
import argparse

import numpy as np



def oscillate_setpoint(interfaces: list[Interface], base_setpoint:np.ndarray, idxs:list[int], 
                       amplitude:float=0.3, period:float=2.0):
    """
    Continuously sinusoidally oscillates specified joint indices

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
        
        print("SETPOINT:", setpoint)

        # TODO: REVIS THIS JANKYNESS
        # interfaces[0].set_joint_positions(setpoint[:7]) # Panda 
        interfaces[1].set_joint_positions(setpoint[7:]) # Tesollo
        time.sleep(0.05)  # ~20Hz update


def main():
    """
    Simple example of arms oscillating (can be bimanual)

    Args:
        interface_str (str): Either "isaacsim" or "panda" ("tesolllo" to come soon)
        parser (ArgumentParser): Argument parser to pass to Isaacsim
    """
    cur_dir = os.path.dirname(__file__)


    
    ## Right Panda
    config_path = os.path.join(cur_dir, "..", "panda", "config", "left_panda_config.yaml")
    panda = PandaInterface.from_yaml(config_path)

    config_path = os.path.join(cur_dir, "..", "tesollo", "config", "left_tesollo_config.yaml")
    tesollo = TesolloInterface.from_yaml(config_path)

    interfaces = [panda, tesollo]

    idxs = [2, 3, 8, 9, 10, 11]
    setpoint = np.zeros(19)  # First 7 are panda, next 12 are tesollo
    setpoint[:7] = np.array([0.0, -np.pi/4,  0.0, -3*np.pi/4, 0.0,  np.pi/2, np.pi/4]) # home for panda
   
    # TODO: MAKE THIS LESS JANKY
    panda.set_joint_positions(setpoint[:7])
    tesollo.set_joint_positions(setpoint[7:])

    osc_thread = threading.Thread(target=oscillate_setpoint, args=(interfaces, setpoint, idxs))
    osc_thread.start()

    # Non-blocking
    # panda.start_loop() 
    tesollo.start_loop()  # blocking

    # Keep the main thread alive
    while True:
        time.sleep(1)


if __name__ == "__main__":

    main()
