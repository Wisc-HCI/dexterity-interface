
from robot_motion_interface.isaacsim.isaacsim_interface import IsaacsimInterface

import os
import numpy as np



def main():
    """
    Simple example of bimanual arms static
    """
    cur_dir = os.path.dirname(__file__)
    config_path = os.path.join(cur_dir, "..", "isaacsim", "config", "isaacsim_config.yaml")

    isaac = IsaacsimInterface.from_yaml(config_path)


    setpoint = np.zeros(38)
    setpoint[:14] = np.array([0.0, 0.0, -np.pi/4, -np.pi/4, 0.0, 0.0,
        -3*np.pi/4, -3*np.pi/4, 0.0, 0.0, np.pi/2, np.pi/2, np.pi/4, np.pi/4])
    isaac.set_joint_positions(setpoint)

    isaac.start_loop()


if __name__ == "__main__":
   
    main()
