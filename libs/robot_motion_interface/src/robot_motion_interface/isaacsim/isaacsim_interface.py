from robot_motion_interface.interface import Interface
from robot_motion_interface.isaacsim.utils.isaac_session import IsaacSession

from enum import Enum
import argparse  # IsaacLab requires using argparse
from typing import TYPE_CHECKING
import os
from pathlib import Path

import numpy as np
import yaml
from pathlib import Path
import torch
from robot_motion import RobotProperties, JointTorqueController



# Imports that need to be loaded after IsaacSession initialized
IMPORTS = [
    "from robot_motion_interface.isaacsim.config.bimanual_arm_env_config import BimanualArmEnvConfig",
    "from isaaclab.envs import ManagerBasedEnv"
]

# This is for type checking
if TYPE_CHECKING:
    from robot_motion_interface.isaacsim.config.bimanual_arm_env_config import BimanualArmEnvConfig
    from isaaclab.envs import ManagerBasedEnv


class IsaacsimControlMode(Enum):
    JOINT_TORQUE = "joint_torque"
    # Future: CART_TORQUE



class IsaacsimInterface(Interface):

    def __init__(self, urdf_path:str, joint_names: list[str], home_joint_positions:np.ndarray, kp: np.ndarray, kd:np.ndarray, control_mode: IsaacsimControlMode,
                 num_envs:int = 1, device: str = 'cuda:0', headless:bool = False, parser: argparse.ArgumentParser = None):
        """
        Isaacsim Interface for running the simulation with accessors for setting
        setpoints of custom controllers.

        Args:
            urdf_path (str): Path to urdf, relative to robot_motion_interface/ (top level).
            joint_names (list[str]): (n_joints) Ordered list of joint names for the robot.
            home_joint_positions (np.ndarray): (n_joints) Default joint positions (rads)
            kp (np.ndarray): (n_joints) Joint proportional gains (array of floats).
            kd (np.ndarray): (n_joints) Joint derivative gains (array of floats).
            control_mode (IsaacsimControlMode): Control mode for the robot (e.g., JOINT_TORQUE).
            num_envs (int): Number of environments to spawn in simulation. Default is 1.
            device (str): Device identifier (e.g., "cuda:0" or "cpu"). Default is "cuda:0".
            headless (bool): If True, run without rendering a viewer window. Default is False.
            parser (ArgumentParser): 
                An existing argument parser to extend. NOTE: If you use parser in a script that calls this one,
                    you WILL need to pass the parser, or this will error. If None, a new parser will be created.
        """
        super().__init__(joint_names)

        self._home_joint_positions = home_joint_positions

        # Isaac Lab uses the parser framework, so adapting our yaml config to this
        if parser:
            self._parser = parser
        else:
            self._parser = argparse.ArgumentParser(description="Isaacsim Interface")
        self._parser.add_argument("--num_envs", type=int)
        self._parser_defaults = {
            'num_envs': num_envs,  
            'device':device, 'headless':headless  # Added by AppLauncher
        }

        self.control_mode_ = control_mode
        self._cur_state = None
        
        cur_dir = os.path.dirname(__file__)
        urdf_resolved_path =  os.path.abspath(os.path.join(cur_dir, "..", "..", "..", urdf_path))
        self._rp = RobotProperties(self._joint_names, urdf_resolved_path)

        if self.control_mode_ == IsaacsimControlMode.JOINT_TORQUE:
            self._controller = JointTorqueController( self._rp, kp, kd, gravity_compensation=True)
        else:
            raise ValueError("Control mode required.")

    
    @classmethod
    def from_yaml(cls, file_path: str, parser: argparse.ArgumentParser = None):
        """
        Construct an IsaacsimInterface instance from a YAML configuration file.

        Args:
            file_path (str): Path to a YAML file containing keys:
                - "urdf_path" (str): Path to urdf, relative to robot_motion_interface/ (top level).
                - "joint_names" (list[str]): (n_joints) Ordered list of joint names for the robot.
                - "home_joint_positions" (np.ndarray): (n_joints) Default joint positions (rads)
                - "kp" (list[float]): (n_joints) Joint proportional gains.
                - "kd" (list[float]): (n_joints) Joint derivative gains.
                - "control_mode" (str): Control mode for the robot (e.g., "joint_torque").
                - "num_envs" (int): Number of environments to spawn in simulation.
                - "device" (str): Device identifier (e.g., "cuda:0", "cpu", etc.).
                - "headless" (bool): Whether to disable the viewer.
            parser (ArgumentParser): An existing argument parser to extend. NOTE: If you use parser in a script that calls this one,
                you WILL need to pass the parser, or this will error. If None, a new parser will be created.
        Returns:
            IsaacsimInterface: initialized interface
        """
        with open(file_path, "r") as f:
            config = yaml.safe_load(f)
        
        relative_urdf_path = config["urdf_path"]
        # File path is provided relative to package directory, so resolve properly
        pkg_dir = Path(__file__).resolve().parents[3]
        urdf_path = str((pkg_dir / relative_urdf_path).resolve())
        joint_names = config["joint_names"]
        home_joint_positions = np.array(config["home_joint_positions"], dtype=float)
        kp = np.array(config["kp"], dtype=float)
        kd = np.array(config["kd"], dtype=float)
        control_mode = IsaacsimControlMode(config["control_mode"])
        num_envs = config["num_envs"]
        device = config["device"]
        headless = config["headless"]

        return cls(urdf_path, joint_names, home_joint_positions, kp, kd, control_mode, num_envs, device, headless, parser)
    

    def start_loop(self):
        """
        Starts the isaacsim simulation loop.
        """
        with IsaacSession(IMPORTS, self._parser, self._parser_defaults) as sess:

            args_cli = sess.args
            simulation_app = sess.app

            env_cfg = BimanualArmEnvConfig()
            env_cfg.scene.num_envs = args_cli.num_envs
            env_cfg.sim.device = args_cli.device
            
            env = ManagerBasedEnv(cfg=env_cfg)
            
            joint_names = self._rp.joint_names()
            expected_names = env.scene.articulations['robot'].data.joint_names
            if list(joint_names) != list(expected_names):
                raise ValueError(
                    f"Joint name mismatch!\nExpected: {expected_names}\nGot: {joint_names}."
                )
            
            joint_efforts = torch.zeros_like(env.action_manager.action)

            env.reset()
            while simulation_app.is_running():

                with torch.inference_mode():

                    obs, _ = env.step(joint_efforts)
                    x = obs["policy"][0]

                    # This puts obs on CPU which is not ideal for speed
                    # TODO: consider pybind torch extension???
                    self._cur_state = (x.detach().to('cpu', dtype=torch.float64).contiguous().view(-1).numpy())
                    step_joint_efforts = self._controller.step(self._cur_state)

                
                    joint_efforts = torch.from_numpy(step_joint_efforts).to(
                        device=env.action_manager.action.device,
                        dtype=env.action_manager.action.dtype,
                    ).unsqueeze(0)  # [1, n] 


            env.close()

    def stop_loop(self):
        """ 
        Stops the background runtime loop
        """
        # TODO

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
        # TODO handle blocking
              
        self._controller.set_setpoint(q)
    

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
        print("HOMING IN ISAACSIM ")

        self.set_joint_positions(q=self._home_joint_positions, blocking=blocking)


    def joint_state(self) -> np.ndarray:
        """
        Get the current joint positions and velocities in order of joint_names.

        Returns:
            (np.ndarray): (n_joints * 2) Current joint angles in radians and joint velocites
                in rad/s
        """
        return self._cur_state



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

       


    

if __name__ == "__main__":

    config_path = Path(__file__).resolve().parents[3] / "config" / "isaacsim_config.yaml"

    isaac = IsaacsimInterface.from_yaml(config_path)
    isaac.start_loop()
    
