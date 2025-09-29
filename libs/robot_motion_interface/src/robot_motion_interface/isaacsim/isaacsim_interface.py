from robot_motion_interface.interface import Interface
from robot_motion_interface.isaacsim.utils.isaac_session import IsaacSession

from enum import Enum
import argparse  # IsaacLab requires using argparse
import threading
from typing import TYPE_CHECKING

import numpy as np
import yaml
import torch
from robot_motion import RobotProperties, JointTorqueController



# Imports that need to be loaded after IsaacSession initialized
# TODO: Read in from yaml????
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

    def __init__(self, joint_names: list[str], kp: np.ndarray, kd:np.ndarray, control_mode: IsaacsimControlMode,
                 num_envs:int = 1, device: str = 'cuda:0', headless:bool = False):
        """
        Isaacsim Interface for running the simulation with accessors for setting
        setpoints of custom controllers.

        Args:
            joint_names (list[str]): (n_joints) Ordered list of joint names for the robot.
            kp (np.ndarray): (n_joints) Joint proportional gains (array of floats).
            kd (np.ndarray): (n_joints) Joint derivative gains (array of floats).
            control_mode (IsaacsimControlMode): Control mode for the robot (e.g., JOINT_TORQUE).
            num_envs (int): Number of environments to spawn in simulation. Default is 1.
            device (str): Device identifier (e.g., "cuda:0" or "cpu"). Default is "cuda:0".
            headless (bool): If True, run without rendering a viewer window. Default is False.
        """

        # Isaac Lab uses the parser framework, so adapting our yaml config to this
        self._parser = argparse.ArgumentParser(description="Isaacsim Interface")
        self._parser.add_argument("--num_envs", type=int)
        self._parser_defaults = {
            'num_envs': num_envs,  
            'device':device, 'headless':headless  # Added by AppLauncher
        }

        self.control_mode_ = control_mode
        self._cur_state = None
        
        self._rp = RobotProperties(joint_names)

        if self.control_mode_ == IsaacsimControlMode.JOINT_TORQUE:
            self._controller = JointTorqueController( self._rp, kp, kd)
        else:
            raise ValueError("Control mode required.")

    
    @classmethod
    def from_yaml(cls, file_path: str):
        """
        Construct an IsaacsimInterface instance from a YAML configuration file.

        Args:
            file_path (str): Path to a YAML file containing keys:
                - "joint_names" (list[str]): (n_joints) Ordered list of joint names for the robot.
                - "kp" (list[float]): (n_joints) Joint proportional gains.
                - "kd" (list[float]): (n_joints) Joint derivative gains.
                - "control_mode" (str): Control mode for the robot (e.g., "joint_torque").
                - "num_envs" (int): Number of environments to spawn in simulation.
                - "device" (str): Device identifier (e.g., "cuda:0", "cpu", etc.).
                - "headless" (bool): Whether to disable the viewer.

        Returns:
            IsaacsimInterface: initialized interface
        """
        with open(file_path, "r") as f:
            config = yaml.safe_load(f)
        
        joint_names = config["joint_names"]
        kp = np.array(config["kp"], dtype=float)
        kd = np.array(config["kd"], dtype=float)
        control_mode = IsaacsimControlMode(config["control_mode"])
        num_envs = config["num_envs"]
        device = config["device"]
        headless =config["headless"]

        return cls(joint_names, kp, kd, control_mode, num_envs, device, headless)
    

    def start_simulation(self):
        """
        Starts the isaacsim simulation loop.
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
        # TODO handle joint names and blocking
              
        self._controller.set_setpoint(q)
    

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


    def _start_loop(self):
        """
        Start the Simulation loop.
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


    

if __name__ == "__main__":
    import os

    cur_dir = os.path.dirname(__file__)
    config_path = os.path.join(cur_dir, "config", "isaacsim_config.yaml")

    isaac = IsaacsimInterface.from_yaml(config_path)
    isaac.start_simulation()
    
