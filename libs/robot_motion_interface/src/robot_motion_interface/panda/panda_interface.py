from robot_motion_interface.robot_motion_interface_pybind import PandaInterface as PandaInterfacePybind
from robot_motion_interface.interface import Interface
from robot_motion.ik.multi_chain_ranged_ik import MultiChainRangedIK
from robot_motion import RobotProperties

import time
from enum import Enum
import numpy as np
import yaml
from pathlib import Path

class PandaControlMode(Enum):
    JOINT_TORQUE = "joint_torque"
    # Future: CART_TORQUE

class PandaInterface(Interface):
    
    def __init__(self, hostname:str, urdf_path:str, ik_settings_path:str, joint_names:list[str], home_joint_positions:np.ndarray,
                 base_frame:str, ee_frames:list[str],kp:np.ndarray, kd:np.ndarray, max_joint_norm_delta:float,
                 control_mode:PandaControlMode=None):
        """
        Python wrapper for C++ Panda Interface.
        Args:
            hostname (str): IP of the Panda
            urdf_path (str): Path to urdf
            ik_settings_path (str): Path to ik settings yaml 
            joint_names (list[str]): (n_joints) Names of all the joints
            home_joint_positions (np.ndarray): (n_joints) Default joint positions (rads)
            base_frame (str): Base frame name for which cartesian poses of end-effector(s) are relative to
            ee_frames (list[str]): (e,) List of frame names for each end-effector
            kp (np.ndarray): (n_joints) Proportional gains for controllers
            kd (np.ndarray): (n_joints) Derivative gains for controllers
            max_joint_norm_delta (float): Caps the Euclidean norm (distance) of the joint delta per control step
                to smooth motion toward the setpoint (in radians). If negative (e.g., -1), the limit is ignored.
            control_mode (PandaControlMode): Control mode for the robot (e.g., JOINT_TORQUE).
        """
        super().__init__(joint_names, home_joint_positions, base_frame, ee_frames)
        self._control_mode = control_mode
        self._panda_interface_cpp = PandaInterfacePybind(hostname, urdf_path, self._joint_names, kp, kd, max_joint_norm_delta)
        self._rp = RobotProperties(self._joint_names, urdf_path) # TODO: get this from c++?
        self._ik_solver = MultiChainRangedIK(ik_settings_path)
    
    @classmethod
    def from_yaml(cls, file_path: str):
        """
        Construct an PandaInterface instance from a YAML configuration file.
        Note: Any relative paths in the yaml are resolved relative to this package 
        directory (robot_motion_interface).

        Args:
            file_path (str): Path to a YAML file containing keys:
                - "hostname" (str): IP of the Panda
                - "urdf_path" (str): Path to urdf, relative to robot_motion_interface/ (top level).
                - "ik_settings_path" (str): Path to ik settings yaml
                - "joint_names" (list[str]): (n_joints) Ordered list of joint names for the robot.
                - "home_joint_positions" (np.ndarray): (n_joints) Default joint positions (rads)
                - "base_frame" (str): Base frame name for which cartesian poses of end-effector(s) are relative to
                - "ee_frames" (list[str]): (e,) List of frame names for each end-effector
                - "kp" (list[float]): (n_joints) Joint proportional gains.
                - "kd" (list[float]): (n_joints) Joint derivative gains.
                - "max_joint_norm_delta" (float): Caps the Euclidean norm (distance) of the joint delta per control step
                to smooth motion toward the setpoint (in radians). If negative (e.g., -1), the limit is ignored.
                - "control_mode" (str): Control mode for the robot (e.g., "joint_torque").

        Returns:
            PandaInterface: initialized interface
        """
        with open(file_path, "r") as f:
            config = yaml.safe_load(f)
        
        hostname = config["hostname"]

        # Relative file path resolve to package directory, so resolve properly
        pkg_dir = Path(__file__).resolve().parents[3]
        relative_urdf_path = config["urdf_path"]
        urdf_path = str((pkg_dir / relative_urdf_path).resolve())
        relative_ik_settings_path = config["ik_settings_path"]
        ik_settings_path = str((pkg_dir / relative_ik_settings_path).resolve())

        joint_names = config["joint_names"]
        home_joint_positions = np.array(config["home_joint_positions"], dtype=float)
        base_frame = config["base_frame"]
        ee_frames = config["ee_frames"]
        kp = np.array(config["kp"], dtype=float)
        kd = np.array(config["kd"], dtype=float)
        max_joint_norm_delta = config["max_joint_norm_delta"]
        control_mode = PandaControlMode(config["control_mode"])

        return cls(hostname, urdf_path, ik_settings_path, joint_names, home_joint_positions,
                   base_frame, ee_frames, kp, kd, max_joint_norm_delta, control_mode)
    


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

        if blocking:
            while(not self.check_reached_target()):
                time.sleep(0.01)



    def set_control_mode(self, control_mode: Enum):
        """
        Set the control mode.

        Args:
            control_mode (Enum): Desired mode.Exact options are implementation-specific.
        """
        ...

    

    def joint_state(self) -> np.ndarray:
        """
        Get the current joint positions and velocities in order of joint_names.

        Returns:
            (np.ndarray): (n_joints * 2) Current joint angles in radians and joint velocities
                in rad/s
        """

        return self._panda_interface_cpp.joint_state()  

        


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

    config_path = Path(__file__).resolve().parents[3] / "config" / "right_panda_config.yaml"
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