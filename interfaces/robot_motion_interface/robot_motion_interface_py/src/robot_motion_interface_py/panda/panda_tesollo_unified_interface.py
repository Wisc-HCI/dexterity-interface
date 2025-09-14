from robot_motion_interface_py.interface import Interface
from robot_motion_interface_py.panda.panda_interface import PandaInterface
from robot_motion_interface_py.tesollo.tesollo_interface import TesolloInterface
 
from enum import Enum
import numpy as np


class PandaTesolloUnifiedInterface(Interface):
    
    def __init__(self):
        """
        Wrapper for using Tesollo attached to Panda (mainly made for unified IK) 
        """
        self.panda = PandaInterface()
        self.tesollo = TesolloInterface()



    def set_joint_positions(self, q:np.ndarray, joint_names:list[str] = None, blocking:bool = False):
        """
        Set the controller's target joint positions at selected joints.

        Args:
            q (np.ndarray): (n_joint_names,) Desired joint angles in radians.
            joint_names (list[str]): (n_joint_names,) Names of joints to command in the same
                order as `q`. If None, assumes q is length of all joints.
            blocking (bool): If True, the call should returns only after the controller
                achieves the target. If False, returns after queuing the request.
        """
        # Todo send panda joints to panda interface and tesollo joints to tesollo interface
        ...
    
    def set_cartesian_pose(self, x:np.ndarray,  base_frame:str = None, ee_frames:list[str] = None, blocking:bool = False):
        """
        Set the controller's target Cartesian pose of one or more end-effectors (EEs).

        Args:
            x (np.ndarray): (7) Target pose in base frame [x, y, z, qx, qy, qz, qw]. 
                            Positions in m, angles in rad. If there is multiple EE frames,
                            will only enforce position, not orientation for all EE joints.
            base_frame (str): Name of base frame that EE pose is relative to. If None,
                defaults to the first joint.
            ee_frames (list[str]): One or more EE frame names to command. If None,
                defaults to the last joint.
            blocking (bool): If True, the call returns only after the controller
                achieves the target. If False, returns after queuing the request.
        """
        # TODO: calculate IK together and call set_joint_position both
        ...

    
    def home(self, blocking:bool = True):
        """
        Move the robot to the predefined home configuration. Blocking.

        Args:
            blocking (bool): If True, the call returns only after the controller
                homes. If False, returns after queuing the home request.
        """
        self.tesollo.home(blocking=False)
        self.panda.home(blocking=blocking)
    

    def joint_positions(self) -> np.ndarray:
        """
        Get the current joint positions in order of joint_names.

        Returns:
            (np.ndarray): (n_joints,) Current joint angles in radians.
        """
        return np.concatenate([self.panda.joint_positions(), self.tesollo.joint_positions()])
        

    def joint_velocities(self) -> np.ndarray:
        """
        Get the current joint velocities in order of joint_names.

        Returns:
            (np.ndarray): (n_joints,) Current joint velocities in radians.
        """
        
        return np.concatenate([self.panda.joint_velocities(), self.tesollo.joint_velocities()])
        


    def cartesian_pose(self, base_frame:str = None, ee_frame:str = None) -> np.ndarray:
        """
        Get the controller's target Cartesian pose of the end-effector (EE).
        Args:
            base_frame (str): Name of base frame that EE pose is relative to. If None,
                defaults to the first joint.
            ee_frames (str): Name of EE frame. If None, defaults to the last joint.
        Returns:
            (np.ndarray): (7) Current pose in base frame [x, y, z, qx, qy, qz, qw]. 
                          Positions in m, angles in rad.
        """

        return self.tesollo.cartesian_pose(base_frame, ee_frame)
        
    
    def joint_names(self) -> list[str]:
        """
        Get the ordered joint names.

        Returns:
            (list[str]): (n_joints) Names of joints
        """
        return self.panda.joint_names() + self.tesollo.joint_names()
    

if __name__ == "__main__":
    panda = PandaTesolloUnifiedInterface()
    