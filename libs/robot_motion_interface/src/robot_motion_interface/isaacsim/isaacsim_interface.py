from robot_motion_interface.interface import Interface
from robot_motion_interface.isaacsim.utils.isaac_session import IsaacSession
from robot_motion.ik.multi_chain_ranged_ik import MultiChainRangedIK

from enum import Enum
import argparse  # IsaacLab requires using argparse
import os

import numpy as np
import yaml
from pathlib import Path
import torch
from robot_motion import RobotProperties, JointTorqueController



class IsaacsimControlMode(Enum):
    JOINT_TORQUE = "joint_torque"
    # Future: CART_TORQUE

class IsaacsimInterface(Interface):

    def __init__(self, urdf_path:str, ik_settings_path:str, joint_names: list[str], home_joint_positions:np.ndarray,
                base_frame:str, ee_frames:list[str], target_tolerance:float,
                kp: np.ndarray, kd:np.ndarray, max_joint_delta:float, control_mode: IsaacsimControlMode,
                num_envs:int = 1, device: str = 'cuda:0', headless:bool = False, parser: argparse.ArgumentParser = None):
        """
        Isaacsim Interface for running the simulation with accessors for setting
        setpoints of custom controllers.

        Args:
            urdf_path (str): Path to urdf, relative to robot_motion_interface/ (top level).
            ik_settings_path (str): Path to ik settings yaml 
            joint_names (list[str]): (n_joints) Ordered list of joint names for the robot.
            home_joint_positions (np.ndarray): (n_joints) Default joint positions (rads)
            base_frame (str): Base frame name for which cartesian poses of end-effector(s) are relative to
            ee_frames (list[str]): (e,) List of frame names for each end-effector
            target_tolerance(float): Threshold (rads) that determines how close the robot's joints must be 
                to the commanded target to count as reached.
            kp (np.ndarray): (n_joints) Joint proportional gains (array of floats).
            kd (np.ndarray): (n_joints) Joint derivative gains (array of floats).
            max_joint_delta (float): Caps the joint delta per control step to smooth motion 
                toward the setpoint (in radians). If negative (e.g., -1), the limit is ignored.
            control_mode (IsaacsimControlMode): Control mode for the robot (e.g., JOINT_TORQUE).
            num_envs (int): Number of environments to spawn in simulation. Default is 1.
            device (str): Device identifier (e.g., "cuda:0" or "cpu"). Default is "cuda:0".
            headless (bool): If True, run without rendering a viewer window. Default is False.
            parser (ArgumentParser): 
                An existing argument parser to extend. NOTE: If you use parser in a script that calls this one,
                you WILL need to pass the parser, or this will error. If None, a new parser will be created.
        """
        super().__init__(joint_names, home_joint_positions, base_frame, ee_frames, target_tolerance)


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

        self._control_mode = control_mode
        self._cur_state = None
        
        cur_dir = os.path.dirname(__file__)
        urdf_resolved_path =  os.path.abspath(os.path.join(cur_dir, "..", "..", "..", urdf_path)) # TODO: TEST Removing
        self._rp = RobotProperties(self._joint_names, urdf_resolved_path)

        if self._control_mode == IsaacsimControlMode.JOINT_TORQUE:
            # TODO: Add joint_norm handleing
            self._controller = JointTorqueController( self._rp, kp, kd, gravity_compensation=True, max_joint_delta=max_joint_delta)
        else:
            raise ValueError("Control mode required.")
        
        self._ik_solver = MultiChainRangedIK(ik_settings_path)
        self._loop_running = False

        self.env = None

    
    @classmethod
    def from_yaml(cls, file_path: str, parser: argparse.ArgumentParser = None):
        """
        Construct an IsaacsimInterface instance from a YAML configuration file. 
        Note: Any relative paths in the yaml are resolved relative to this package 
        directory (robot_motion_interface).

        Args:
            file_path (str): Path to a YAML file containing keys:
                - "urdf_path" (str): Path to urdf, relative to robot_motion_interface/ (top level).
                - "ik_settings_path" (str): Path to ik settings yaml
                - "joint_names" (list[str]): (n_joints) Ordered list of joint names for the robot.
                - "home_joint_positions" (np.ndarray): (n_joints) Default joint positions (rads)
                - "base_frame" (str): Base frame name for which cartesian poses of end-effector(s) are relative to
                - "ee_frames" (list[str]): (e,) List of frame names for each end-effector
                - "target_tolerance" (float): Threshold (rads) that determines how close the robot's 
                    joints must be to the commanded target to count as reached.
                - "kp" (list[float]): (n_joints) Joint proportional gains.
                - "kd" (list[float]): (n_joints) Joint derivative gains.
                - "max_joint_delta" (float): Caps the Euclidean norm (distance) of the joint delta per control step
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
        target_tolerance = config["target_tolerance"]

        kp = np.array(config["kp"], dtype=float)
        kd = np.array(config["kd"], dtype=float)
        max_joint_delta = config["max_joint_delta"]
        control_mode = IsaacsimControlMode(config["control_mode"])
        num_envs = config["num_envs"]
        device = config["device"]
        headless = config["headless"]

        return cls(urdf_path, ik_settings_path, joint_names, home_joint_positions, base_frame, ee_frames,
                   target_tolerance,
                   kp, kd, max_joint_delta, control_mode, num_envs, device, headless, parser)
    

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
              
        self._controller.set_setpoint(q)
        
        if blocking:
            self._block_until_reached_target()

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
            (np.ndarray): (n_joints * 2) Current joint angles in radians and joint velocites
                in rad/s
        """
        return self._cur_state


    def check_loop(self) -> bool:
        """ 
        Check if the simulation is running.

        Returns:
            (bool): True if loop is running, false if not running.
        """
        return self._loop_running


    def start_loop(self):
        """
        Starts the isaacsim simulation loop.
        """

        # Start Isaac Session to manage Kit life cycle
        with IsaacSession(self._parser, self._parser_defaults) as sess:
            
            # Must be imported after Kit loaded
            from isaaclab.envs import ManagerBasedEnv

            args_cli = sess.args
            simulation_app = sess.app

            # 1. Configure environment (can be overridden)
            env_cfg = self._setup_env_cfg(args_cli)
            self.env = ManagerBasedEnv(cfg=env_cfg)
            
            # 2. Post-environment setup (can be overridden)
            self._post_env_creation(self.env)

            self.env.reset()
            self._loop_running = True

            while simulation_app.is_running():
                with torch.inference_mode():
                    # 3. Step during loop (can be overridden)
                    obs = self._step(self.env)

                    # 4. Process observation
                    self._post_step(self.env, obs)

            self.env.close()
            self._loop_running = False
            

    def stop_loop(self):
        """ 
        Stops the background runtime loop
        """
        # TODO
    
    
    #####################################################################
    # Simulation Hooks: can be overwritten or extended by child classes #
    #####################################################################

    def _setup_env_cfg(self, args_cli: argparse.Namespace) -> "ManagerBasedEnvCfg":
        """
        (Hook) Creates and configures the environment
        Args:
            args_cli (argparse.Namespace): Command-line arguments parsed by IsaacSession.

        Returns:
            (ManagerBasedEnvCfg): The configuration used to initialize the environment.
        """

        # Must be imported after kit loaded
        from robot_motion_interface.isaacsim.config.bimanual_arm_env_config import BimanualArmEnvConfig
        
        env_cfg = BimanualArmEnvConfig()
        env_cfg.scene.num_envs = args_cli.num_envs
        env_cfg.sim.device = args_cli.device

        return env_cfg


    def _post_env_creation(self, env: "ManagerBasedEnv"):
        """
        (Hook) Called immediately after the environment is created. Subclasses can 
        extend or overwrite to spawn assets, configure scene entities, etc.

        Args:
            env (ManagerBasedEnv): The active simulation environment.
        """
                
        joint_names = self._rp.joint_names()
        expected_names = env.scene.articulations['robot'].data.joint_names
        if list(joint_names) != list(expected_names):
            raise ValueError(
                f"Joint name mismatch!\nExpected: {expected_names}\nGot: {joint_names}."
            )
        
        self._joint_efforts = torch.zeros_like(env.action_manager.action)


    def _step(self, env: "ManagerBasedEnv") -> dict:
        """
        (Hook) Called every simulation tick to step the simulation.

        Args:
            env (ManagerBasedEnv): The active simulation environment.
        Returns:
            (dict): The raw observation dictionary from the environment.
        """
        obs, _ = env.step(self._joint_efforts)
        
        return obs

    def _post_step(self, env: "ManagerBasedEnv", obs: dict):
        """
        (Hook) Called after simulation _step. Subclasses can 
        extend or overwrite set joint state, etc.
        Args:
            env (ManagerBasedEnv): The active simulation environment.
            obs (dict): The raw observation dictionary from the environment.
        """

        # TODO: Make this more abstract and make child more specific????
        x = obs["policy"][0]

        # This puts obs on CPU which is not ideal for speed
        # TODO: consider pybind torch extension???
        self._cur_state = (x.detach().to('cpu', dtype=torch.float64).contiguous().view(-1).numpy())
        step_joint_efforts = self._controller.step(self._cur_state)

        self._joint_efforts = torch.from_numpy(step_joint_efforts).to(
            device=env.action_manager.action.device,
            dtype=env.action_manager.action.dtype,
        ).unsqueeze(0)  # [1, n] 



if __name__ == "__main__":

    config_path = Path(__file__).resolve().parents[3] / "config" / "isaacsim_config.yaml"

    isaac = IsaacsimInterface.from_yaml(config_path)
    isaac.start_loop()
    
