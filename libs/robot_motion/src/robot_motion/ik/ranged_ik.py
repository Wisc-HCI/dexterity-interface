from robot_motion.ik.ik import IK
import numpy as np

# TODO: Function comments

class RangedIK(IK):
    def __init__(self):
        ...
    
    def solve(x:np.ndarray, base_frame:str, ee_frame:str) -> np.ndarray:
        ...
    