from robot_motion_interface.utils.array_utils import get_partial_update_reference_map, partial_update

from abc import abstractmethod
from enum import Enum
import numpy as np


class Interface:
    def __init__(self, joint_names:list[str], home_joint_positions:np.ndarray, base_frame:str, ee_frames:list[str]):
        """
        Parent interface for running different robot interfaces

        Args:
            joint_names (list[str]): (n_joints) Ordered list of joint names for the robot.
            home_joint_positions (np.ndarray): (n_joints) Default joint positions (rads)
            base_frame (str): Base frame name for which cartesian poses of end-effector(s) are relative to
            ee_frames (list[str]): (e,) List of frame names for each end-effector
        """
        # For partial joint/cartesian updates
        self._joint_names = joint_names
        self._home_joint_positions = home_joint_positions
        self._base_frame = base_frame
        self._ee_frames = ee_frames

        self._joint_reference_map = get_partial_update_reference_map(joint_names)

        if self._ee_frames:
            self._ee_reference_map = get_partial_update_reference_map(ee_frames)

        # Filled in by children
        self._ik_solver = None
        self._rp = None

    def set_cartesian_pose(self, x_list:np.ndarray, ee_frames:list[str] = None, blocking:bool = False):
        """
        Set the controller's target Cartesian pose of one or more end-effectors (EEs).

        Args:
            x_list (np.ndarray): (e, 7) List of target poses [x, y, z, qx, qy, qw, qz] * c in m, angles in rad. One target
                pose per ee_frame
            ee_frames (list[str]): (e) One or more EE frame names to command. If None,
                defaults to the last joint.

            blocking (bool): If True, the call returns only after the controller
                achieves the target. If False, returns after queuing the request.
        """

        # TODO: handle blocking
        x_list = self._partial_to_full_cartesian_positions(x_list, ee_frames)

        q, joint_order = self._ik_solver.solve(x_list)

        self.set_joint_positions(q, joint_order, blocking)

    def cartesian_pose(self, ee_frames:str = None) -> tuple[np.ndarray, list[str]]:
        """
        Get the controller's target Cartesian pose of the end-effector (EE).
        Args:
            ee_frames (str): (e,) Name of EE frame. If None, defaults to all EEs
        Returns:
            (np.ndarray): (e, 7) List of current poses for each EE in base frame [x, y, z, qx, qy, qz, qw]. 
                          Positions in m, angles in rad.
            (list[str]): (e,) List of names of EE frames
        """
        if not ee_frames:
            ee_frames = self._ee_frames

        cur_joint_state = self.joint_state()

        if cur_joint_state is None or cur_joint_state.size == 0:
            cur_joint_state = self._home_joint_positions
        poses = []
        for frame in ee_frames:
            cart_pose = self._rp.forward_kinematics(cur_joint_state, self._base_frame, frame)
            poses.append(cart_pose)

        return np.vstack(poses), ee_frames
    
   
    def joint_names(self) -> list[str]:
        """
        Get the ordered joint names.

        Returns:
            (list[str]): (n_joints) Names of joints
        """
        return self._joint_names
    
    def home(self, blocking:bool = True):
        """
        Move the robot to the predefined home configuration. Blocking.

        Args:
            blocking (bool): If True, the call returns only after the controller
                homes. If False, returns after queuing the home request.
        """
        self.set_joint_positions(q=self._home_joint_positions, blocking=blocking)

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
    def set_control_mode(self, control_mode: Enum):
        """
        Set the control mode.

        Args:
            control_mode (Enum): Desired mode.Exact options are implementation-specific.
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
        
        # Default to home position if joint_state not given yet
        cur_state = self.joint_state()
        cur_q = cur_state[:n] if ( cur_state is not None and cur_state.size > 0) else self._home_joint_positions

        return partial_update(cur_q, self._joint_reference_map, q, joint_names) 


    def _partial_to_full_cartesian_positions(self, x_list:np.ndarray, ee_frames:str = None) -> np.ndarray:
        """
        If there are multiple End-effectors, converts setpoint for a subset of end-effectors to
        the full list of end-effectors by filling in the undefined setpoints with the current pose.

        Args:
            x_list (np.ndarray): (e,7) Array of target poses [x, y, z, qx, qy, qz, qw] (first 3 in m, 
                last 4 in quaternions). Each pose corresponds to each ee_frames
            ee_frames (str): (e,) List of names of EE frames
        Returns:
            np.ndarray: (7,) Full array of cartesian pose values with updated values from x inserted 
                at positions corresponding to cartesian_order (if provided).
        Raises:
            ValueError: If lengths of x and cartesian_order do not match the expected sizes.
        """

        if not self._ee_frames or not self._base_frame:
            raise ValueError(f"base_frame and/or ee_frames were not set in the constructor. Can not execute _partial_to_full_cartesian_positions.")

        n_x = 7
        for x in x_list:
            if len(x) != n_x:
                raise ValueError(f"Each cartesian pose in x must be length {n_x}")
        
        n_ee = len(self._ee_frames)

        if not ee_frames and n_ee != len(x_list):
            raise ValueError(f"If ee_frames is not passed, x must be length {n_ee}")
        
        if not ee_frames:
            return x
        
        cur_x, _ = self.cartesian_pose()

        return partial_update(cur_x, self._ee_reference_map, x_list, ee_frames) 

