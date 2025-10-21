from robot_motion.ik.ik import IK
import numpy as np
import python_wrapper as RelaxedIKRust

# TODO: Function comments

class RangedIK(IK):
    def __init__(self):
        print("RangedIK initialized")
    
    def solve(x:np.ndarray, base_frame:str, ee_frame:str) -> np.ndarray:
        print(f"Solving IK for position {x} from {base_frame} to {ee_frame}")
    