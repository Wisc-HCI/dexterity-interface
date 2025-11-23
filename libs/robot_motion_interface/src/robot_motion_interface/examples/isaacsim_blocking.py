   
from robot_motion_interface.isaacsim.isaacsim_interface import IsaacsimInterface

import time
from pathlib import Path
import numpy as np
import threading

def run_blocking_targets(interface: IsaacsimInterface):
    """
    Set a couple blocking cartesian targets, one after the other.

    Args:
        interface (IsaacsimInterface): The interface Isaacsim interface instance
    """

    time.sleep(30) # Wait for isaacsim to load. TODO: Update this when merge object branch
    ee = ['left_delto_offset_link', 'right_delto_offset_link']

    print("Starting first target.")
    left_goal_1 = np.array([-0.2, 0.2, 0.1, 0.707, 0.707, 0, 0])
    right_goal_1 = np.array([0.2, 0.2, 0.1, 0.707, 0.707, 0, 0])
    interface.set_cartesian_pose([left_goal_1, right_goal_1], ee, blocking=True)
    print("Arrived at first target.")

    left_goal_2 = np.array([-0.4, 0.2, 0.4, 0.707, 0.707, 0, 0])
    right_goal_2 = np.array([0.4, 0.2, 0.4, 0.707, 0.707, 0, 0])
    interface.set_cartesian_pose([left_goal_2, right_goal_2], ee, blocking=True)
    print("Arrived at second target.")

    left_goal_3 = np.array([-0.1, 0.2, 0.4, 0.707, 0.707, 0, 0])
    right_goal_3 = np.array([0.1, 0.2, 0.4, 0.707, 0.707, 0, 0])
    interface.set_cartesian_pose([left_goal_3, right_goal_3], ee, blocking=True)
    print("Arrived at final target.")


def main():
    """
    Simple example of static bimanual arms in Isaacsim, solved with 
    cartesian IK.
    """
    config_dir = Path(__file__).resolve().parents[3] / "config"
    config_path = config_dir / "isaacsim_config.yaml"

    isaac = IsaacsimInterface.from_yaml(config_path)




    # Start at default position
    setpoint = np.zeros(38)
    setpoint[:14] = np.array([0.0, 0.0, -np.pi/4, -np.pi/4, 0.0, 0.0,
        -3*np.pi/4, -3*np.pi/4, 0.0, 0.0, np.pi/2, np.pi/2, np.pi/4, np.pi/4])
    isaac.set_joint_positions(setpoint)

    print("Starting THREAD")
    targets_thread = threading.Thread(target=run_blocking_targets, args=(isaac,))
    targets_thread.start()

    isaac.start_loop()




if __name__ == "__main__":
   
    main()
