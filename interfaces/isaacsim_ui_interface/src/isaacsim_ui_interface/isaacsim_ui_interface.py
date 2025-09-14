
from robot_motion_interface.isaacsim.isaacsim_interface import IsaacsimInterface
from enum import Enum
import numpy as np

class IsaacsimUIInterface(IsaacsimInterface):
    def __init__(self):
        """
        Isaacsim Interface for running the simulation with accessors for using IsaacSim more 
        like a GUI (moving objects, resetting the environment, etc.).
        """
        self._start_loop()

    def freeze(self, env_ids=None):
        """
        TODO
        """
        ...


    def unfreeze(self, env_ids=None):
        """
        TODO
        """
        ...


    def place_object(self, env_ids=None):
        """
        TODO
        """
        ...

    def move_object(self, env_ids=None):
        """
        TODO
        """
        ...



if __name__ == "__main__":
    isaac = IsaacsimUIInterface()
    
