from robot_motion_interface_py.interface import Interface
from enum import Enum
import numpy as np


class PandaInterface(Interface):
    
    def __init__(self):
        """
        Python wrapper for C++ Panda Interface.
        """
        self._start_loop()


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
        ...

    def set_control_mode(self, control_mode: Enum):
        """
        Set the control mode.

        Args:
            control_mode (Enum): Desired mode.Exact options are implementation-specific.
        """
        ...
    
    def home(self, blocking:bool = True):
        """
        Move the robot to the predefined home configuration. Blocking.

        Args:
            blocking (bool): If True, the call returns only after the controller
                homes. If False, returns after queuing the home request.
        """
        ...
    

    def joint_positions(self) -> np.ndarray:
        """
        Get the current joint positions in order of joint_names.

        Returns:
            (np.ndarray): (n_joints,) Current joint angles in radians.
        """
        ...

    def joint_velocities(self) -> np.ndarray:
        """
        Get the current joint velocities in order of joint_names.

        Returns:
            (np.ndarray): (n_joints,) Current joint velocities in radians.
        """
        ...


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
        ...
        
    
    def joint_names(self) -> list[str]:
        """
        Get the ordered joint names.

        Returns:
            (list[str]): (n_joints) Names of joints
        """
        ...
    

    ########################## Private ##########################
    def _write_joint_torques(self, tau:np.ndarray):
        """
        Writes torque commands directly to motor.

        Args:
            tau (np.ndarray): (n_joints,) Commanded joint torques [N·m].
        """
        ...
    

    def _write_joint_positions(self, q:np.ndarray):
        """
        Write position commands directly to motor.

        Args:
            q (np.ndarray):  (n_joints,) Commanded joint positions [rad].
        """
        ...


    def _start_loop(self):
        """
        Start the background runtime (e.g. for control loop and/or simulation loop).
        """
        ...


if __name__ == "__main__":
    panda = PandaInterface()
    