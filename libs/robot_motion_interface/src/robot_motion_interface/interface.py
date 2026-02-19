from robot_motion_interface.utils.array_utils import get_partial_update_reference_map, partial_update
from robot_motion_interface.utils.trajectory_utils import interpolate_cartesian_trajectory

from abc import abstractmethod
import time
from enum import Enum
import numpy as np
import threading


class Interface:
    def __init__(self, joint_names:list[str], home_joint_positions:np.ndarray, base_frame:str, ee_frames:list[str],
                 target_tolerance:float):
        """
        Parent interface for running different robot interfaces

        Args:
            joint_names (list[str]): (n_joints) Ordered list of joint names for the robot.
            home_joint_positions (np.ndarray): (n_joints) Default joint positions (rads)
            base_frame (str): Base frame name for which cartesian poses of end-effector(s) are relative to
            ee_frames (list[str]): (e,) List of frame names for each end-effector
            target_tolerance(float): Threshold (rads) that determines how close the robot's joints must be to the commanded target to count as reached.
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


        # Need to track this to prevent drift when doing partial updates
        # and also to check target position
        self._joint_setpoint = home_joint_positions

        # Used to check if reached target position
        self._target_tolerance = target_tolerance
        self._previous_joint_difference_norm = None
        self._best_joint_difference_norm = None
        self._stall_count = 0

        # Used to interrupt movement blocking
        self._blocking_event = threading.Event()

    def check_reached_target(self, allow_stall:bool=False, stall_threshold:int=3, stall_delta:float=0.01) -> bool:
        """
        Check if the robot reached the target set by set_joint_positions
        or set_cartesian_pose. Uses target_tolerance on norm of joints.
        Args:
            allow_stall (bool): If this is true, will return true when the
                robot has stalled (hasn't reached target but stopped moving).
                This is useful for grippers grasping objects.
            stall_threshold (int): Number of consecutive checks where the norm hasn't improved
                on the best seen by more than stall_delta before declaring a stall.
                Scale with check frequency (e.g. at dt=0.02: 3).
            stall_delta (float): Dead-band tolerance (rad, norm) — improvement smaller than
                this is ignored to absorb sim oscillations. Default: 0.01.
        Returns:
            (bool): True if robot has reached target, else False
        """
        if self._joint_setpoint is None:
            print("WARNING: No target set with set_joint_positions or set_cartesian_pose. check_reached_target() will always return True.")
            return True

        cur_joint_position= self.joint_state()
        n = len(self._joint_names)
        if cur_joint_position is None or cur_joint_position.size == 0:
            print("WARNING: No joint state received. check_reached_target() returns False.")
            return False
        else:
            cur_joint_position = cur_joint_position[:n]

        self._previous_joint_state_checked = cur_joint_position

        difference = cur_joint_position - self._joint_setpoint
        difference_norm = np.linalg.norm(difference)
        is_target_reached = difference_norm < self._target_tolerance

        if not is_target_reached and allow_stall and self._best_joint_difference_norm is not None:
            if difference_norm < self._best_joint_difference_norm - stall_delta:
                # Made meaningful progress — reset
                self._best_joint_difference_norm = difference_norm
                self._stall_count = 0
            else:
                # No meaningful progress toward target
                self._stall_count += 1
                if self._stall_count >= stall_threshold:
                    print("WARNING: Robot stalling.")
                    is_target_reached = True

        if is_target_reached:
            self._best_joint_difference_norm = None
            self._stall_count = 0
        elif self._best_joint_difference_norm is None:
            self._best_joint_difference_norm = difference_norm

        self._previous_joint_difference_norm = difference_norm

        return is_target_reached




    def _block_until_reached_target(self):
        """
        Called internally to block until reached target.
        Call interrupt_movement() to exit before reaching target.
        TODO: Add timeout
        """
        self._blocking_event.set() 
        while(not self.check_reached_target() and self._blocking_event.is_set()):
                time.sleep(0.01)
        self._blocking_event.clear()
    

    def interrupt_movement(self):
        """
        Stop blocking or non-blocking set_cartesian_pose() or set_joint_positions() by
        setting setpoint to current joint position.
        """
        self._blocking_event.clear()
        n = len(self.joint_names())
        cur_joint_state = self.joint_state()[:n]

        # Set setpoint to current state to interrupt target
        self.set_joint_positions(cur_joint_state)


    def set_cartesian_pose(self, x_list:np.ndarray, ee_frames:list[str] = None, blocking:bool = False):
        """
        Set the controller's target Cartesian pose of one or more end-effectors (EEs).

        Args:
            x_list (np.ndarray): (e, 7) List of target poses [x, y, z, qx, qy, qz, qw] * e in m, angles in rad. 
                One target pose per ee_frame
            ee_frames (list[str]): (e) One or more EE frame names to command (must be subset of those
                set in the constructo). If None, defaults to the EEs set in the constructor.

            blocking (bool): If True, the call returns only after the controller
                achieves the target. If False, returns after queuing the request.
        """
        
        x_list = self._partial_to_full_cartesian_positions(x_list, ee_frames)

        q, joint_order = self._ik_solver.solve(x_list)

        self.set_joint_positions(q, joint_order, blocking)

    

    def cartesian_pose(self, ee_frames:list[str] = None) -> tuple[np.ndarray, list[str]]:
        """
        Get the controller's target Cartesian pose of the end-effector (EE).
        Args:
            ee_frames (list[str]): (e,) Names of EE frames. If None, defaults to all EEs
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
        else:
            n = len(self._joint_names)
            cur_joint_state = cur_joint_state[:n]
        poses = self._forward_kinematics(cur_joint_state, self._base_frame, ee_frames)

        return poses, ee_frames


    def cartesian_trajectory(self, goal_poses: np.ndarray, dt: float, velocity: float,
                             ee_frames: list[str] = None) -> tuple[list[np.ndarray], list[str]]:
        """
        Generate interpolated Cartesian trajectories from the current EE poses to goal poses.

        Args:
            goal_poses (np.ndarray): (e, 7) Target poses [x, y, z, qx, qy, qz, qw] in m/rad.
                One per ee_frame.
            dt (float): Time step between trajectory points in seconds.
            velocity (float): Desired linear velocity in m/s.
            ee_frames (list[str]): (e,) EE frame names. If None, defaults to all EE frames.

        Returns:
            (list[np.ndarray]): e arrays each of shape (N_i, 7) of interpolated poses per EE.
                Each trajectory may have a different length depending on the distance to its goal.
            (list[str]): (e,) List of names of EE frames
        """
        if ee_frames is None:
            ee_frames = self._ee_frames

        goal_poses = np.atleast_2d(goal_poses)
        if len(goal_poses) != len(ee_frames):
            raise ValueError(f"goal_poses length ({len(goal_poses)}) must match ee_frames length ({len(ee_frames)})")

        cur_poses, _ = self.cartesian_pose(ee_frames)

        trajectories = []
        for start_pose, goal_pose in zip(cur_poses, goal_poses):
            traj = interpolate_cartesian_trajectory(start_pose, goal_pose, dt, velocity)
            trajectories.append(traj)

        return trajectories, ee_frames


    def joint_names(self) -> list[str]:
        """
        Get the ordered joint names.

        Returns:
            (list[str]): (n_joints) Names of joints
        """
        return self._joint_names
    

    def home(self, blocking:bool = False):
        """
        Move the robot to the predefined home configuration. Blocking.

        Args:
            blocking (bool): If True, the call returns only after the controller
                homes. If False, returns after queuing the home request.
        """

        if self._ik_solver:
            # TODO: reset when use set_joint_position too
            self._ik_solver.reset()

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
            self._joint_setpoint = q
            return q

        n_update = len(joint_names)
        if n_q != n_update:
            raise ValueError(f"Length of q ({n_q}) does not match length of joint_names ({n_update})")
        

        full_q = partial_update(self._joint_setpoint, self._joint_reference_map, q, joint_names) 
        
        self._joint_setpoint = full_q

        return full_q


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
        
        cur_x_list = self._forward_kinematics(self._joint_setpoint, self._base_frame, self._ee_frames)

        full_x_list = partial_update(cur_x_list, self._ee_reference_map, x_list, ee_frames) 

        return full_x_list



    def _forward_kinematics(self, joint_positions:np.ndarray, base_frame: str, ee_frames: list[str]):
        """
        Calculates EE positions given the joint state from the base_frame to each ee_frame.

        Args:
            joint_positions (np.ndarray): Current joint positions of the robot (must be in order of config)
            base_frame (str): Names of base frame.
            ee_frames (list[str]): (e,) Names of EE frames.
        Returns:
            (np.ndarray): (e, 7) List of current poses for each EE in base frame [x, y, z, qx, qy, qz, qw]. 
                          Positions in m, angles in rad.
        """
        poses = []
        for frame in ee_frames:
            cart_pose = self._rp.forward_kinematics(joint_positions, base_frame, frame)
            poses.append(cart_pose)
        return np.vstack(poses)