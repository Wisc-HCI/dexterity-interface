from robot_motion_interface.robot_motion_interface_pybind import PandaInterface as PandaInterfacePybind

from robot_motion_interface.interface import Interface
from enum import Enum
import numpy as np
import yaml
from pathlib import Path

class PandaControlMode(Enum):
    JOINT_TORQUE = "joint_torque"
    # Future: CART_TORQUE

class PandaInterface(Interface):
    
    def __init__(self, hostname:str, urdf_path:str, joint_names:list[str], home_joint_positions:np.ndarray,
                 kp:np.ndarray, kd:np.ndarray, control_mode:PandaControlMode=None):
        """
        Python wrapper for C++ Panda Interface.
        Args:
            hostname (str): IP of the Panda
            urdf_path (str): Path to urdf
            joint_names (list[str]): (n_joints) Names of all the joints
            home_joint_positions (np.ndarray): (n_joints) Default joint positions (rads)
            kp (np.ndarray): (n_joints) Proportional gains for controllers
            kd (np.ndarray): (n_joints) Derivative gains for controllers
            control_mode (PandaControlMode): Control mode for the robot (e.g., JOINT_TORQUE).
        """
        super().__init__(joint_names)
        self._home_joint_positions = home_joint_positions
        self._control_mode = control_mode
        self._panda_interface_cpp = PandaInterfacePybind(hostname, urdf_path, self._joint_names, kp, kd)
    
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

        Returns:
            PandaInterface: initialized interface
        """
        with open(file_path, "r") as f:
            config = yaml.safe_load(f)
        
        hostname = config["hostname"]

        relative_urdf_path = config["urdf_path"]
        # File path is provided relative to package directory, so resolve properly
        pkg_dir = Path(__file__).resolve().parents[3]
        urdf_path = str((pkg_dir / relative_urdf_path).resolve())

        joint_names = config["joint_names"]
        home_joint_positions = np.array(config["home_joint_positions"], dtype=float)
        kp = np.array(config["kp"], dtype=float)
        kd = np.array(config["kd"], dtype=float)
        control_mode = PandaControlMode(config["control_mode"])

        return cls(hostname, urdf_path, joint_names, home_joint_positions, kp, kd, control_mode)
    


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
        q = self._partial_to_full_joint_positions(q, joint_names)
        # TODO: handle blocking
        self._panda_interface_cpp.set_joint_positions(q)
    
    def set_cartesian_pose(self, x:np.ndarray, cartesian_order:list[str] = None, base_frame:str = None, ee_frames:list[str] = None, blocking:bool = False):
        """
        Set the controller's target Cartesian pose of one or more end-effectors (EEs).

        Args:
            x (np.ndarray): (c, ) Target pose in base frame Positions in m, angles in rad. If there is multiple EE frames,
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

        return self._panda_interface_cpp.joint_state()  


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
        return self._joint_names
    



    def start_loop(self):
        """
        Start control loop
        """
        self._panda_interface_cpp.start_loop()

    
    def stop_loop(self):
        """ 
        Stops the background runtime loop
        """
        self._panda_interface_cpp.stop_loop()

if __name__ == "__main__":
    import os

    cur_dir = os.path.dirname(__file__)
    config_path = os.path.join(cur_dir, "config", "left_panda_config.yaml")

    panda = PandaInterface.from_yaml(config_path)
    try:
        panda.start_loop()  
        panda.home()
        while(True):  # Keep thread running
            ...
    except (KeyboardInterrupt):
        print("\nStopping Panda.")
    finally:
        panda.stop_loop()