from robot_motion_interface.interface import Interface
from robot_motion_interface.isaacsim.utils.isaac_session import IsaacSession

from enum import Enum
import argparse  # IsaacLab requires using argparse
from typing import TYPE_CHECKING

import numpy as np
import yaml
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

        # TODO: Add these to yaml
        # joint_names = [
        #     # Left arm
        #     "left_panda_joint1", "left_panda_joint2", "left_panda_joint3", "left_panda_joint4",
        #     "left_panda_joint5", "left_panda_joint6", "left_panda_joint7", "left_panda_joint8",
        #     # Left gripper
        #     "left_F1M1", "left_F1M2", "left_F1M3", "left_F1M4", "left_TIP1",
        #     "left_F2M1", "left_F2M2", "left_F2M3", "left_F2M4", "left_TIP2",
        #     "left_F3M1", "left_F3M2", "left_F3M3", "left_F3M4", "left_TIP3",
        #     # Right arm
        #     "right_panda_joint1", "right_panda_joint2", "right_panda_joint3", "right_panda_joint4",
        #     "right_panda_joint5", "right_panda_joint6", "right_panda_joint7", "right_panda_joint8",
        #     # Right gripper
        #     "right_F1M1", "right_F1M2", "right_F1M3", "right_F1M4", "right_TIP1",
        #     "right_F2M1", "right_F2M2", "right_F2M3", "right_F2M4", "right_TIP2",
        #     "right_F3M1", "right_F3M2", "right_F3M3", "right_F3M4", "right_TIP3"
        # ]

        joint_names = [
            # Left arm
            "left_panda_joint1", "left_panda_joint2", "left_panda_joint3", "left_panda_joint4",
            "left_panda_joint5", "left_panda_joint6", "left_panda_joint7", 
            # Left gripper
            "left_F1M1", "left_F1M2", "left_F1M3", "left_F1M4", 
            "left_F2M1", "left_F2M2", "left_F2M3", "left_F2M4", 
            "left_F3M1", "left_F3M2", "left_F3M3", "left_F3M4", 
            # Right arm
            "right_panda_joint1", "right_panda_joint2", "right_panda_joint3", "right_panda_joint4",
            "right_panda_joint5", "right_panda_joint6", "right_panda_joint7",
            # Right gripper
            "right_F1M1", "right_F1M2", "right_F1M3", "right_F1M4", 
            "right_F2M1", "right_F2M2", "right_F2M3", "right_F2M4", 
            "right_F3M1", "right_F3M2", "right_F3M3", "right_F3M4", 
        ]



        rp = RobotProperties(joint_names)
        kp = np.ones(rp.n_joints(), dtype=np.float64) * 500.0 
        kd = np.ones(rp.n_joints(), dtype=np.float64) * 20.0
        self._controller = JointTorqueController(rp, kp, kd)
        setpoint = np.zeros(rp.n_joints())
        self._controller.set_setpoint(setpoint)
        
        self._start_loop()
        
    
    @classmethod
    def from_yaml(cls, file_path: str):
        """
        Construct an IsaacsimInterface instance from a YAML configuration file.

        TODO: Consider removing bc py config files do similar job.

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

            env_cfg = BimanualArmEnvConfig()
            env_cfg.scene.num_envs = args_cli.num_envs
            env_cfg.sim.device = args_cli.device
            
            env = ManagerBasedEnv(cfg=env_cfg)

            count = 0
            joint_efforts = torch.zeros_like(env.action_manager.action)
            while simulation_app.is_running():

                with torch.inference_mode():
                    # if count % 300 == 0:
                    #     count = 0
                    #     env.reset()
                    
                    obs, _ = env.step(joint_efforts)
                    # TODO: Handle more than 1 env
                    x = obs["policy"][0]

                    # This puts obs on CPU which is not ideal for speed
                    # TODO: consider pybind torch extension???
                    state = (x.detach().to('cpu', dtype=torch.float64).contiguous().view(-1).numpy())
                    step_joint_efforts = self._controller.step(state)
                
                    joint_efforts = torch.from_numpy(step_joint_efforts).to(
                        device=env.action_manager.action.device,
                        dtype=env.action_manager.action.dtype,
                    ).unsqueeze(0)  # [1, n]  <-- important


                    print("STATE:", state)
                    print("Torques:", step_joint_efforts)

                                        
                    count += 1

            env.close()


    

if __name__ == "__main__":
    import os

    cur_dir = os.path.dirname(__file__)
    config_path = os.path.join(cur_dir, "config", "isaacsim_config.yaml")

    isaac = IsaacsimInterface.from_yaml(config_path)
    
