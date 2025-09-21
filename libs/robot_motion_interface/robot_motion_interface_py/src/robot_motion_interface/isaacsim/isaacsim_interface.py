
"""
python3 libs/robot_motion_interface/robot_motion_interface_py/src/robot_motion_interface/isaacsim/isaacsim_interface.py
"""

from robot_motion_interface.interface import Interface
from robot_motion_interface.isaacsim.utils.isaac_session import IsaacSession

from enum import Enum
import argparse  # IsaacLab requires using argparse
from typing import TYPE_CHECKING

import numpy as np
import yaml
import torch


# Imports that need to be loaded after IsaacSession initialized
IMPORTS = [
    "from robot_motion_interface.isaacsim.config.bimanual_arm_env_config import CartpoleEnvCfg",
    "from isaaclab.envs import ManagerBasedEnv"
]

# This is for type checking
if TYPE_CHECKING:
    from robot_motion_interface.isaacsim.config.bimanual_arm_env_config import CartpoleEnvCfg
    from isaaclab.envs import ManagerBasedEnv



class IsaacsimInterface(Interface):

    def __init__(self, num_envs:int = 1, device: str = 'cuda:0', headless:bool = False):
        """
        Isaacsim Interface for running the simulation with accessors for setting
        setpoints of custom controllers.

        Args:
            num_envs (int): Number of environments to spawn in simulation.
            device (str): The device identifier (e.g. "cuda:0" or "cpu").
            headless (bool): If True, run without rendering a viewer window.
        """

        # Isaac Lab uses the parser framework, so adapting our yaml config to this
        self._parser = argparse.ArgumentParser(description="Isaacsim Interface")
        self._parser.add_argument("--num_envs", type=int)
        self._parser_defaults = {
            'num_envs': num_envs,  
            'device':device, 'headless':headless  # Added by AppLauncher
        }

        self._start_loop()
        
    
    @classmethod
    def from_yaml(cls, file_path: str):
        """
        Construct an IsaacsimInterface instance from a YAML configuration file.

        Args:
            file_path (str): Path to a YAML file containing keys:
                - "num_envs" (int): number of environments
                - "device" (str): device string ("cuda:0", "cpu", etc.)
                - "headless" (bool): whether to disable the viewer

        Returns:
            IsaacsimInterface: initialized interface
        """
        with open(file_path, "r") as f:
            config = yaml.safe_load(f)

        return cls(config["num_envs"], config["device"], config["headless"])
    

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
    

    def joint_positions(self) -> np.ndarray:
        """
        Get the current joint positions in order of joint_names.

        Returns:
            (np.ndarray): (n_joints,) Current joint angles in radians.
        """
        ...

    def joint_velocities(self) -> np.ndarray:
        """
        Get the current joint velocities in order of joint_names.

        Returns:
            (np.ndarray): (n_joints,) Current joint velocities in radians.
        """
        ...


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

            env_cfg = CartpoleEnvCfg()
            env_cfg.scene.num_envs = args_cli.num_envs
            env_cfg.sim.device = args_cli.device
            
            # setup base environment
            env = ManagerBasedEnv(cfg=env_cfg)

            # simulate physics
            count = 0

            while simulation_app.is_running():

                with torch.inference_mode():
                    # reset
                    if count % 300 == 0:
                        count = 0
                        env.reset()
                        print("-" * 80)
                        print("[INFO]: Resetting environment...")
                    # sample random actions
                    joint_efforts = torch.randn_like(env.action_manager.action)
                    # step the environment
                    obs, _ = env.step(joint_efforts)
                    # print current orientation of pole
                    print("OBSERVATION", obs["policy"][0][1].item())
                    # update counter
                    count += 1

            # close the environment
            env.close()


    

if __name__ == "__main__":
    import os

    cur_dir = os.path.dirname(__file__)
    config_path = os.path.join(cur_dir, "config", "isaacsim_config.yaml")

    isaac = IsaacsimInterface.from_yaml(config_path)
    
