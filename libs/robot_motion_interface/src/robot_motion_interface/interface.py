from robot_motion_interface.utils.array_utils import get_partial_update_reference_map, partial_update

from abc import abstractmethod
from enum import Enum
import numpy as np


class Interface:
    def __init__(self, joint_names:list[str]):

        # For partial joint/cartesian updates
        self._joint_names = joint_names
        self._joint_reference_map = get_partial_update_reference_map(joint_names)
        self._cartesian_reference_map = get_partial_update_reference_map(["x", "y", "z", "qx", "qy", "qy", "qw"])

    
    @abstractmethod
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
    
    @abstractmethod
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

    @abstractmethod
    def set_control_mode(self, control_mode: Enum):
        """
        Set the control mode.

        Args:
            control_mode (Enum): Desired mode.Exact options are implementation-specific.
        """
        ...
    
    @abstractmethod
    def home(self, blocking:bool = True):
        """
        Move the robot to the predefined home configuration. Blocking.

        Args:
            blocking (bool): If True, the call returns only after the controller
                homes. If False, returns after queuing the home request.
        """
        ...
    

    @abstractmethod
    def joint_state(self) -> np.ndarray:
        """
        Get the current joint positions and velocities in order of joint_names.

        Returns:
            (np.ndarray): (n_joints * 2) Current joint angles in radians and joint velocities
                in rad/s
        """
        ...


    @abstractmethod
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
        
    
    @abstractmethod
    def joint_names(self) -> list[str]:
        """
        Get the ordered joint names.

        Returns:
            (list[str]): (n_joints) Names of joints
        """
        ...
    
    @abstractmethod
    def start_loop(self):
        """
        Start the background runtime (e.g. for control loop and/or simulation loop).
        """
        ...

    @abstractmethod
    def stop_loop(self):
        """ 
        Stops the background runtime loop
        """
        ...

    ########################## Private ##########################
    
    def _partial_to_full_joint_positions(self,  q:np.ndarray, joint_names:list[str] = None) -> np.ndarray:
        """
        Converts a partial joint position array to a full joint position array.

        Args:
            q (np.ndarray): (b,) Array of joint position values.  
            joint_names (list[str]): (b) List of joint names corresponding to positions in q.  
        Returns:
            np.ndarray: (n,) Full array of joint positions with updated values from q inserted 
            at positions corresponding to joint_names (if provided).
        Raises:
            ValueError: If lengths of q and joint_names do not match the expected sizes.
        """

        n = len(self._joint_names)
        n_q = q.size

        if not joint_names and n_q != n:
            raise ValueError(f"If joint_names is not passed, q must be length {n}")
        
        if not joint_names:
            return q
        
        n_update = len(joint_names)
        if n_q != n_update:
            raise ValueError(f"Length of q ({n_q}) does not match length of joint_names ({n_update})")
        
        cur_q = self.joint_state()[:n]
        return partial_update(cur_q, self._joint_reference_map, q, joint_names) 


    def _partial_to_full_cartesian_positions(self, x:np.ndarray, cartesian_names:list[str] = None):
        # TODO
        return x

