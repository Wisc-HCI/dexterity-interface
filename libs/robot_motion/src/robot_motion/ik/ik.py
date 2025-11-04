from abc import abstractmethod
import numpy as np

class IK:
    def __init__(self):
        ...
    
    @abstractmethod
    def solve(x:np.ndarray, base_frame:str, ee_frame:str) -> tuple[np.ndarray, list[str]]:
        """
        Solves for joint positions based on cartesian pose
        Args:
            x (np.ndarray): (7,) cartesian pose [x, y, z, qx, qy, qz, qw] 
            base_frame (str): The name of the base reference frame in which the pose `x` 
                            is expressed.
            ee_frame (str): The name of the end-effector frame to be solved for.
        Returns:
            (np.ndarray): (n,) The joint solution
            (list[str]): Name of each joint

        """
        ...
    