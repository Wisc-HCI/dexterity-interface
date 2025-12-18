"""
Oscillates robot joints using Panda or IsaacSim interface.

Usage:
    python -m robot_motion_interface.examples.oscillate_robots --backend panda
    python -m robot_motion_interface.examples.oscillate_robots --backend isaacsim
"""

from robot_motion_interface.interface import Interface



import os
import time
import threading
import argparse
from pathlib import Path

import numpy as np



def oscillate_setpoint(interface: Interface, base_setpoint:np.ndarray, idxs:list[int], 
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

        interface.set_joint_positions(setpoint)
        time.sleep(0.05)  # ~20Hz update

def main(interface_str:str, parser: argparse.ArgumentParser = None):
    """
    Simple example of arms oscillating (can be bimanual)

    Args:
        interface_str (str): Either "isaacsim" or "panda" ("tesolllo" to come soon)
        parser (ArgumentParser): Argument parser to pass to Isaacsim
    """
    config_dir = Path(__file__).resolve().parents[3] / "config"

    if (interface_str == "isaacsim"):
        # Imported conditionally so that unessary dependencies aren't required
        from robot_motion_interface.isaacsim.isaacsim_interface import IsaacsimInterface
        config_path = config_dir / "isaacsim_config.yaml"
        interface = IsaacsimInterface.from_yaml(config_path, parser)
        idxs = [4, 5, 6, 7, 30, 31, 32, 33, 34, 35, 36, 37]
        setpoint = np.zeros(38)
        setpoint[:14] = np.array([0.0, 0.0, -np.pi/4, -np.pi/4, 0.0, 0.0,
            -3*np.pi/4, -3*np.pi/4, 0.0, 0.0, np.pi/2, np.pi/2, np.pi/4, np.pi/4])
    
    elif (interface_str == "panda"):
        # Imported conditionally so that unessary dependencies aren't required
        from robot_motion_interface.panda.panda_interface import PandaInterface
        config_path = config_dir / "right_panda_config.yaml"
        interface = PandaInterface.from_yaml(config_path)
        idxs = [2, 3]
        setpoint = np.array([0.0, -np.pi/4,  0.0, -3*np.pi/4, 0.0,  np.pi/2, np.pi/4]) # home 
    else:
        raise ValueError(f"Unsupported interface: {interface_str}")

    
    interface.set_joint_positions(setpoint)

    osc_thread = threading.Thread(target=oscillate_setpoint, args=(interface, setpoint, idxs))
    osc_thread.start()

    interface.start_loop()

    try: 
        while(True):
            time.sleep(0.1)
    except (KeyboardInterrupt):
        print("\nStopping Interface.")
    finally:
        interface.stop_loop()  



if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Run joint oscillation demo for Panda or Isaacsim.")
    parser.add_argument("--interface", type=str, choices=["panda", "isaacsim"], required=True,
                        help="Choose 'panda' for PandaInterface, 'isaacsim' for IsaacsimInterface.")
    args = parser.parse_args()
    main(args.interface, parser)
