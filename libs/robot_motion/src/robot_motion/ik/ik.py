from abc import abstractmethod
import numpy as np

# TODO: Function comments

class IK:
    def __init__(self):
        ...
    
    @abstractmethod
    def solve(x:np.ndarray, base_frame:str, ee_frame:str) -> np.ndarray:
        ...
    