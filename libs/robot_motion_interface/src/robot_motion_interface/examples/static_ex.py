
from robot_motion_interface.isaacsim.isaacsim_interface import IsaacsimInterface

from pathlib import Path
import numpy as np



def main():
    """
    Simple example of static bimanual arms in Isaacsim
    """
    config_dir = Path(__file__).resolve().parents[3] / "config"
    config_path = config_dir / "isaacsim_config.yaml"

    isaac = IsaacsimInterface.from_yaml(config_path)


    setpoint = np.zeros(38)
    setpoint[:14] = np.array([0.0, 0.0, -np.pi/4, -np.pi/4, 0.0, 0.0,
        -3*np.pi/4, -3*np.pi/4, 0.0, 0.0, np.pi/2, np.pi/2, np.pi/4, np.pi/4])
    isaac.set_joint_positions(setpoint)

    isaac.start_loop()


if __name__ == "__main__":
   
    main()
