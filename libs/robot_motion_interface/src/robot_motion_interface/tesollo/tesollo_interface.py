from robot_motion_interface.interface import Interface

from robot_motion_interface.robot_motion_interface_pybind import TesolloDg3fInterface as TesolloDg3fInterfacePybind
import time
from enum import Enum
import numpy as np
import yaml
from pathlib import Path

class TesolloControlMode(Enum):
    JOINT_TORQUE = "joint_torque"
    # Future: CART_TORQUE

# TODO: UDPATE NAME TO TesolloDG3F
class TesolloInterface(Interface):
    
    def __init__(self, ip:str, port:int, joint_names:list[str], home_joint_positions:np.ndarray,
                 kp:np.ndarray, kd:np.ndarray,  target_tolerance:float, 
                 control_loop_frequency:float, control_mode:TesolloControlMode=None):
        """
        Tesollo Interface for running controlling the Tesollo hand.
        Args:
            ip (str): IP of the left Tesollo
            port (int): Port of the Panda
            joint_names (list[str]): (n_joints) Names of all the joints
            home_joint_positions (np.ndarray): (n_joints) Default joint positions (rads)
            target_tolerance(float): Threshold (rads) that determines how close the robot's joints 
                must be to the commanded target to count as reached.
            kp (np.ndarray): (n_joints) Proportional gains for controllers
            kd (np.ndarray): (n_joints) Derivative gains for controllers
            control_loop_frequency (float): Frequency that control loop runs at (Hz). Default: 500 hz
            control_mode (TesolloControlMode): Control mode for the robot (e.g., JOINT_TORQUE).
        """
        super().__init__(joint_names, home_joint_positions, None, None, target_tolerance)  # No frames for cart position needed
        self._control_mode = control_mode
        self._tesollo_interface_cpp = TesolloDg3fInterfacePybind(ip, port, self._joint_names, kp, kd, control_loop_frequency)
    
    @classmethod
    def from_yaml(cls, file_path: str):
        """
        Construct an TesolloInterface instance from a YAML configuration file.

        Args:
            file_path (str): Path to a YAML file containing keys:
                - "ip" (str): IP of the left Tesollo
                - "port" (int): Port of the Panda
                - "joint_names" (list[str]): (n_joints) Ordered list of joint names for the robot.
                - "home_joint_positions" (np.ndarray): (n_joints) Default joint positions (rads)
                - "target_tolerance" (float): Threshold (rads) that determines how close the robot's joints must be 
                        to the commanded target to count as reached.
                - "kp" (list[float]): (n_joints) Joint proportional gains.
                - "kd" (list[float]): (n_joints) Joint derivative gains.
                - "control_loop_frequency" (float): Frequency that control loop runs at (Hz). Default: 500 hz
                - "control_mode" (str): Control mode for the robot (e.g., "joint_torque").
                

        Returns:
            TesolloInterface: initialized interface
        """
        with open(file_path, "r") as f:
            config = yaml.safe_load(f)
        
        ip = config["ip"]
        port = config["port"]
        joint_names = config["joint_names"]
        home_joint_positions = np.array(config["home_joint_positions"], dtype=float)
        target_tolerance = config["target_tolerance"]
        kp = np.array(config["kp"], dtype=float)
        kd = np.array(config["kd"], dtype=float)
        control_mode = TesolloControlMode(config["control_mode"])
        control_loop_frequency = config["control_loop_frequency"]
        

        return cls(ip, port, joint_names, home_joint_positions, target_tolerance, 
                   kp, kd, control_loop_frequency, control_mode)
    


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
        self._tesollo_interface_cpp.set_joint_positions(q)
        if blocking:
            while(not self.check_reached_target()):
                time.sleep(0.01)
        
    
    def set_cartesian_pose(self,  *args, **kwargs):
        """
        Not implemented for Tesollo since so many joints
        """
        print("WARNING: set_cartesian_pose() is not implemented for Tesollo because of its joint complexity.")

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
        return self._tesollo_interface_cpp.joint_state()  



    def cartesian_pose(self,  *args, **kwargs):
        """
        Not implemented for Tesollo since so many joints
        """
        print("WARNING: cartesian_pose() is not implemented for Tesollo because of its joint complexity.")
        return np.array([])
        
    
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

    config_path = Path(__file__).resolve().parents[3] / "config" / "left_tesollo_config.yaml"
    
    tesollo = TesolloInterface.from_yaml(config_path)
    try: 
        tesollo.start_loop()
        tesollo.home()
        while(True):  # Keep thread running
            ...
    except (KeyboardInterrupt):
        print("\nStopping Tesollo.")
    finally:
        tesollo.stop_loop()


