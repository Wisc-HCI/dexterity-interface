
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

    # wrist_goal_left = np.array([0.0, 0.2, 0.4, 0.707, 0.707, 0, 0])
    # wrist_goal_right = np.array([0.2, 0.2, 0.4, 0.0,   1.57,  0.0, 1.0])
    
    x = [wrist_goal_left, wrist_goal_right]
    isaac.set_cartesian_pose(x)

    isaac.start_loop()


if __name__ == "__main__":
   
    main()


# Q [ 2.15734012  0.63296362 -1.89509969 -1.97122594  0.61895302  1.76014296 -1.50987322 
#   -2.01288003 -1.7628      2.16636476 -2.78523487 -2.8973 1.62311001 -0.22353371]

