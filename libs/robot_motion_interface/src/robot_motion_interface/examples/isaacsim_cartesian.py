
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

    wrist_goal_left = np.array([0, .2, .5, 0, 1.57, 0, 1])
    wrist_goal_right = np.array([0, .2, .5, 0, -1.57, 0, 1])
    
    x = [wrist_goal_left, wrist_goal_right]
    isaac.set_cartesian_pose(x)

    isaac.start_loop()


if __name__ == "__main__":
   
    main()
