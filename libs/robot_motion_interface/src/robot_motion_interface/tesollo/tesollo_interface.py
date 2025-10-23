from robot_motion_interface.interface import Interface

from ..robot_motion_interface_pybind import TesolloDg3fInterface as TesolloDg3fInterfacePybind
from enum import Enum
import numpy as np
import yaml

class TesolloControlMode(Enum):
    JOINT_TORQUE = "joint_torque"
    # Future: CART_TORQUE

# TODO: UDPATE NAME TO TesolloDG3F
class TesolloInterface(Interface):
    
    def __init__(self, ip:str, port:int, joint_names:list[str], home_joint_positions:np.ndarray,
                 kp:np.ndarray, kd:np.ndarray, control_loop_frequency:float, control_mode:str):
        """
        Tesollo Interface for running controlling the Tesollo hand.
        TODO: UPDATE
        """
        super().__init__(joint_names)
        self._home_joint_positions = home_joint_positions
        self._control_mode = control_mode
        self._tesollo_interface_cpp = TesolloDg3fInterfacePybind(ip, port, self._joint_names, kp, kd, control_loop_frequency)
    
    @classmethod
    def from_yaml(cls, file_path: str):
        """
        Construct an PandaInterface instance from a YAML configuration file.

        Args:
            file_path (str): Path to a YAML file containing keys:
                - "hostname" (str): IP of the Panda
                - "urdf_path" (str): Path to urdf, relative to robot_motion_interface/ (top level).
                - "joint_names" (list[str]): (n_joints) Ordered list of joint names for the robot.
                - "home_joint_positions" (np.ndarray): (n_joints) Default joint positions (rads)
                - "kp" (list[float]): (n_joints) Joint proportional gains.
                - "kd" (list[float]): (n_joints) Joint derivative gains.
                - "control_mode" (str): Control mode for the robot (e.g., "joint_torque").

                TODO: UPDATE
        Returns:
            PandaInterface: initialized interface
        """
        with open(file_path, "r") as f:
            config = yaml.safe_load(f)
        
        ip = config["ip"]
        port = config["port"]
        joint_names = config["joint_names"]
        home_joint_positions = np.array(config["home_joint_positions"], dtype=float)
        kp = np.array(config["kp"], dtype=float)
        kd = np.array(config["kd"], dtype=float)
        control_loop_frequency = config["control_loop_frequency"]
        control_mode = TesolloControlMode(config["control_mode"])

        return cls(ip, port, joint_names, home_joint_positions, kp, kd, control_loop_frequency, control_mode)
    


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
        # TODO: handle blocking, joint names
        q = self._partial_to_full_joint_positions(q, joint_names)
        self._tesollo_interface_cpp.set_joint_positions(q)
        
    
    def set_cartesian_pose(self, x:np.ndarray, cartesian_order:list[str] = None, base_frame:str = None, ee_frames:list[str] = None, blocking:bool = False):
        """
        Set the controller's target Cartesian pose of one or more end-effectors (EEs).

        Args:
            x (np.ndarray): (c, ) Target pose in base frame. Positions in m, angles in rad. If there is multiple EE frames,
                            will only enforce position, not orientation for all EE joints.
            cartesian_order (list[str]): (c, ). If none, the joint order must be ["x", "y", "z", "qx", "qy", "qz", "qw"]
            base_frame (str): Name of base frame that EE pose is relative to. If None,
                defaults to the first joint.
            ee_frames (list[str]): One or more EE frame names to command. If None,
                defaults to the last joint.
            blocking (bool): If True, the call returns only after the controller
                achieves the target. If False, returns after queuing the request.
        """
        x = self._partial_to_full_cartesian_positions(x, cartesian_order, base_frame, ee_frames)
        # TODO: implementation, blocking

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
        self.set_joint_positions(q=self._home_joint_positions, blocking=blocking)
    

    def joint_state(self) -> np.ndarray:
        """
        Get the current joint positions and velocities in order of joint_names.

        Returns:
            (np.ndarray): (n_joints * 2) Current joint angles in radians and joint velocities
                in rad/s
        """
        return self._tesollo_interface_cpp.joint_state()  



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
        # TODO
        ...

        
    
    def joint_names(self) -> list[str]:
        """
        Get the ordered joint names.

        Returns:
            (list[str]): (n_joints) Names of joints
        """
        return self._joint_names
    
    
    def start_loop(self):
        """
        Start control loop
        """
        self._tesollo_interface_cpp.start_loop()

    
    def stop_loop(self):
        """
        Safely stop the control loop
        """
        self._tesollo_interface_cpp.stop_loop()

if __name__ == "__main__":
    import os

    cur_dir = os.path.dirname(__file__)
    config_path = os.path.join(cur_dir, "config", "left_tesollo_config.yaml")

    tesollo = TesolloInterface.from_yaml(config_path)

