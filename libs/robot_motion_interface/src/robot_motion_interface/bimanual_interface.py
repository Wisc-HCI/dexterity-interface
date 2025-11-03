from robot_motion_interface.interface import Interface
from robot_motion_interface.panda.panda_interface import PandaInterface
from robot_motion_interface.tesollo.tesollo_interface import TesolloInterface
from robot_motion.ik.multi_chain_ranged_ik import MultiChainRangedIK

from pathlib import Path
import numpy as np
import yaml

class BimanualInterface(Interface):
    
    def __init__(self, enable_left:bool, enable_right:bool,
                 urdf_path:str, ik_settings_path:str, base_frame:str, ee_frames:list[str],

                 panda_home_joint_positions:np.ndarray, 
                 panda_kp:np.ndarray, panda_kd:np.ndarray, 

                 tesollo_home_joint_positions:np.ndarray, 
                 tesollo_control_loop_frequency:float, tesollo_kp:np.ndarray, tesollo_kd:np.ndarray, 

                 left_panda_hostname:str = None, left_panda_joint_names:list[str] = [], 
                 right_panda_hostname:str = None, right_panda_joint_names:list[str] = [], 

                 left_tesollo_ip:str = None, left_tesollo_port:int = None, left_tesollo_joint_names:list[str] = [], 
                 right_tesollo_ip:str = None, right_tesollo_port:int = None, right_tesollo_joint_names:list[str] = []
                 ):
        """
        Wrapper for using both pandas and both tesollos. Similar to Isaacsim Interface
        Args:
            enable_left(bool): True if using the left panda/tesollo
            enable_right(bool): True if using the right panda/tesollo

            urdf_path (str): Path to urdf
            ik_settings_path (str): Path to ik settings yaml 
            base_frame (str): Base frame name for which cartesian poses of end-effector(s) are relative to
            ee_frames (list[str]): (e,) List of frame names for each end-effector

            panda_home_joint_positions (np.ndarray): (n_joints) Default joint positions (rads)
            panda_kp (np.ndarray): (n_joints) Proportional gains for controllers
            panda_kd (np.ndarray): (n_joints) Derivative gains for controllers

            tesollo_home_joint_positions (np.ndarray): (n_joints) Default joint positions (rads)
            tesollo_control_loop_frequency (float): Frequency that control loop runs at (Hz). Default: 500 hz
            tesollo_kp (np.ndarray): (n_joints) Proportional gains for controllers
            tesollo_kd (np.ndarray): (n_joints) Derivative gains for controllers

            left_panda_hostname (str): IP of the left Panda
            left_panda_joint_names (list[str]): (n_joints) Names of all the joints

            right_panda_hostname (str): IP of the Panda
            right_panda_joint_names (list[str]): (n_joints) Names of all the joints

            left_tesollo_ip (str): IP of the left Tesollo
            left_tesollo_port (int): Port of the Tesollo
            left_tesollo_joint_names (list[str]): (n_joints) Names of all the joints
           
            right_tesollo_ip (str): IP of the right Tesollo
            right_tesollo_port (int): Port of the Tesollo
            right_tesollo_joint_names (list[str]): (n_joints) Names of all the joints
        """

        self._enable_left = enable_left
        self._enable_right = enable_right

        # TODO: Figure out how to not initialized 3 IK instances
        if not self._enable_left and not self._enable_right:
            raise ValueError("Must set enable_left, enable_right, or both to True.")
        if self._enable_left:
            self._panda_left = PandaInterface(left_panda_hostname, urdf_path, ik_settings_path, left_panda_joint_names, 
                panda_home_joint_positions, base_frame, ee_frames, panda_kp, panda_kd)
            self._tesollo_left = TesolloInterface(left_tesollo_ip, left_tesollo_port, left_tesollo_joint_names, 
                tesollo_home_joint_positions, tesollo_kp,tesollo_kd, tesollo_control_loop_frequency)
            self._n_panda = len(self._panda_left.joint_names())
            self._n_tesollo = len(self._tesollo_left.joint_names())
        
        if self._enable_right:
            self._panda_right = PandaInterface(right_panda_hostname, urdf_path, ik_settings_path, right_panda_joint_names, 
                panda_home_joint_positions, base_frame, ee_frames, panda_kp, panda_kd)
            self._tesollo_right = TesolloInterface(right_tesollo_ip, right_tesollo_port, right_tesollo_joint_names, 
                tesollo_home_joint_positions, tesollo_kp,tesollo_kd, tesollo_control_loop_frequency)
            
            self._n_panda = len(self._panda_right.joint_names())
            self._n_tesollo = len(self._tesollo_right.joint_names())

        joint_names = left_panda_joint_names + left_tesollo_joint_names + right_panda_joint_names + right_tesollo_joint_names
        home_joint_positions = np.concatenate([ panda_home_joint_positions, tesollo_home_joint_positions,
            panda_home_joint_positions, tesollo_home_joint_positions])
        super().__init__(joint_names, home_joint_positions, base_frame, ee_frames)

        self._ik_solver = MultiChainRangedIK(ik_settings_path)

    @classmethod
    def from_yaml(cls, file_path: str):
        """
        Construct an BimanualInterface instance from a YAML configuration file.        
        Note: Any relative paths in the yaml are resolved relative to this package 
        directory (robot_motion_interface).

        Args:
            file_path (str): Path to a YAML file containing keys:
                - "enable_left"(bool): True if using the left panda/tesollo
                - "enable_right"(bool): True if using the right panda/tesollo

                - "urdf_path" (str): Path to urdf (relative to `robot_motion_interface/` directory) 
                - "ik_settings_path" (str): Path to ik settings yaml
                - "base_frame" (str): Base frame name for which cartesian poses of end-effector(s) are relative to
                - "ee_frames" (list[str]): (e,) List of frame names for each end-effector

                - "panda_home_joint_positions" (np.ndarray): (n_joints) Default joint positions (rads)
                - "panda_kp" (np.ndarray): (n_joints) Proportional gains for controllers
                - "panda_kd" (np.ndarray): (n_joints) Derivative gains for controllers
                
                - "tesollo_home_joint_positions" (np.ndarray): (n_joints) Default joint positions (rads)
                - "tesollo_control_loop_frequency" (float): Frequency that control loop runs at (Hz). Default: 500 hz
                - "tesollo_kp" (np.ndarray): (n_joints) Proportional gains for controllers
                - "tesollo_kd" (np.ndarray): (n_joints) Derivative gains for controllers

                - "left_panda_hostname" (str): IP of the left Panda
                - "left_panda_joint_names" (list[str]): (n_joints) Names of all the joints

                - "right_panda_hostname" (str): IP of the Panda
                - "right_panda_joint_names" (list[str]): (n_joints) Names of all the joints

                - "left_tesollo_ip" (str): IP of the left Tesollo
                - "left_tesollo_port" (int): Port of the Panda
                - "left_tesollo_joint_names" (list[str]): (n_joints) Names of all the joints
           
                - "right_tesollo_ip" (str): IP of the right Tesollo
                - "right_tesollo_port" (int): Port of the Panda
                - "right_tesollo_joint_names" (list[str]): (n_joints) Names of all the joints

        Returns:
            TesolloInterface: initialized interface
        """
        with open(file_path, "r") as f:
            config = yaml.safe_load(f)
        
        enable_left = bool(config["enable_left"])
        enable_right = bool(config["enable_right"])

        # Relative file path resolve to package directory, so resolve properly
        pkg_dir = Path(__file__).resolve().parents[2]
        relative_urdf_path = config["urdf_path"]
        urdf_path = str((pkg_dir / relative_urdf_path).resolve())
        relative_ik_settings_path = config["ik_settings_path"]
        ik_settings_path = str((pkg_dir / relative_ik_settings_path).resolve())
        base_frame = config["base_frame"]
        ee_frames = config["ee_frames"]

        panda_home_joint_positions = np.array(config["panda_home_joint_positions"], dtype=float)
        panda_kp = np.array(config["panda_kp"], dtype=float)
        panda_kd = np.array(config["panda_kd"], dtype=float)
        
        tesollo_home_joint_positions = np.array(config["tesollo_home_joint_positions"], dtype=float)
        tesollo_control_loop_frequency = config["tesollo_control_loop_frequency"]
        tesollo_kp = np.array(config["tesollo_kp"], dtype=float)
        tesollo_kd = np.array(config["tesollo_kd"], dtype=float)

        # Optional
        left_panda_hostname = config.get("left_panda_hostname")
        left_panda_joint_names = config.get("left_panda_joint_names", [])
        right_panda_hostname = config.get("right_panda_hostname")
        right_panda_joint_names = config.get("right_panda_joint_names", [])

        left_tesollo_ip = config.get("left_tesollo_ip")
        left_tesollo_port = config.get("left_tesollo_port")
        left_tesollo_joint_names = config.get("left_tesollo_joint_names", [])
        right_tesollo_ip = config.get("right_tesollo_ip")
        right_tesollo_port = config.get("right_tesollo_port")
        right_tesollo_joint_names = config.get("right_tesollo_joint_names", [])

        return cls(enable_left, enable_right, urdf_path, ik_settings_path, base_frame, ee_frames,
                 panda_home_joint_positions, 
                 panda_kp, panda_kd, tesollo_home_joint_positions, tesollo_control_loop_frequency, 
                 tesollo_kp, tesollo_kd, left_panda_hostname, left_panda_joint_names, right_panda_hostname, 
                 right_panda_joint_names, left_tesollo_ip, left_tesollo_port, left_tesollo_joint_names, 
                 right_tesollo_ip, right_tesollo_port, right_tesollo_joint_names)
    


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
        
        idx = 0
        if self._enable_left:
            self._panda_left.set_joint_positions(q[:idx + self._n_panda])
            idx += self._n_panda

            self._tesollo_left.set_joint_positions(q[idx : idx + self._n_tesollo])
            idx += self._n_tesollo

        if self._enable_right:
            self._panda_right.set_joint_positions(q[idx : idx + self._n_panda])
            idx += self._n_panda

            self._tesollo_right.set_joint_positions(q[idx:])
            idx += self._n_tesollo
            

    

    
    def home(self, blocking:bool = True):
        """
        Move the robot to the predefined home configuration. Blocking.

        Args:
            blocking (bool): If True, the call returns only after the controller
                homes. If False, returns after queuing the home request.
        """
        if self._enable_left and self._enable_right:
            self._tesollo_left.home(blocking=False)
            self._panda_left.home(blocking=False)
            self._tesollo_right.home(blocking=False)
            self._panda_right.home(blocking=blocking)
        elif self._enable_left:
            self._tesollo_left.home(blocking=False)
            self._panda_left.home(blocking=blocking)
        elif self._enable_right:
            self._tesollo_right.home(blocking=False)
            self._panda_right.home(blocking=blocking)

        # TODO: Handle blocking better

    

    def joint_state(self) -> np.ndarray:
        """
        Get the current joint positions and velocities in order of joint_names.

        Returns:
            (np.ndarray): (n_joints * 2) Current joint angles in radians and joint velocities
                in rad/s. [panda_left_pos;  tesollo_left_pos; panda_right_pos; tesollo_right_pos; 
                panda_left_vel; tesollo_left_vel; panda_right_vel; tesollo_right_vel]
        """

        state = []

        # Concat position
        if self._enable_left:
            panda_left_joint_state = self._panda_left.joint_state()
            tesollo_left_joint_state = self._tesollo_left.joint_state()
            state.extend([ 
                panda_left_joint_state[:self._n_panda],
                tesollo_left_joint_state[:self._n_tesollo]
            ])
                                    
        if self._enable_right:
            panda_right_joint_state = self._panda_right.joint_state()
            tesollo_right_joint_state = self._tesollo_right.joint_state()
            state.extend([ 
                panda_right_joint_state[:self._n_panda],
                tesollo_right_joint_state[:self._n_tesollo]
            ])

        # Concat velocity
        if self._enable_left:
            state.extend([ 
                panda_left_joint_state[self._n_panda:],
                tesollo_left_joint_state[self._n_tesollo:]
            ])
                                    
        if self._enable_right:
            state.extend([
                panda_right_joint_state[self._n_panda:],
                tesollo_right_joint_state[self._n_tesollo:]
            ])

        return np.concatenate(state)


        
    
    def joint_names(self) -> list[str]:
        """
        Get the ordered joint names.

        Returns:
            (list[str]): (n_joints) Names of joints
        """
        return self._joint_names
    
    def start_loop(self):
        """
        Start control loops
        """
        if self._enable_left:
            self._tesollo_left.start_loop()
            self._panda_left.start_loop()
        if self._enable_right:
            self._tesollo_right.start_loop()
            self._panda_right.start_loop()

    
    def stop_loop(self):
        """
        Safely stop the control loops
        """
        if self._enable_left:
            self._tesollo_left.stop_loop()
            self._panda_left.stop_loop()
        if self._enable_right:
            self._tesollo_right.stop_loop()
            self._panda_right.stop_loop()


if __name__ == "__main__":
    pkg_dir = Path(__file__).resolve().parents[2]
    config_path = pkg_dir / "config" / "bimanual_arm_config.yaml"

    arms = BimanualInterface.from_yaml(config_path)

    try: 
        arms.start_loop()  
        arms.home() 
        while(True):
            ...
    except (KeyboardInterrupt):
        print("\nStopping Tesollo.")
    finally:
        arms.stop_loop()
