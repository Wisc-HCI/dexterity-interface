
from robot_motion_interface.isaacsim.isaacsim_interface import IsaacsimInterface, IsaacsimControlMode
import argparse  # IsaacLab requires using argparse

import numpy as np
from pathlib import Path

class IsaacsimObjectInterface(IsaacsimInterface):
    def __init__(self, urdf_path:str, ik_settings_path:str, joint_names: list[str], home_joint_positions:np.ndarray,
                base_frame:str, ee_frames:list[str],
                kp: np.ndarray, kd:np.ndarray, control_mode: IsaacsimControlMode,
                num_envs:int = 1, device: str = 'cuda:0', headless:bool = False, parser: argparse.ArgumentParser = None):
        """
        Isaacsim Interface  extension for running the simulation with accessors for using IsaacSim with object
        interactions (moving objects, resetting the environment, etc.).

        Args:
            urdf_path (str): Path to urdf, relative to robot_motion_interface/ (top level).
            ik_settings_path (str): Path to ik settings yaml 
            joint_names (list[str]): (n_joints) Ordered list of joint names for the robot.
            home_joint_positions (np.ndarray): (n_joints) Default joint positions (rads)
            base_frame (str): Base frame name for which cartesian poses of end-effector(s) are relative to
            ee_frames (list[str]): (e,) List of frame names for each end-effector
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
        
        super().__init__(urdf_path, ik_settings_path, joint_names, home_joint_positions,
            base_frame, ee_frames, kp, kd, control_mode, num_envs, device, headless, parser)

    def freeze(self):
        """
        TODO
        """
        ...


    def unfreeze(self):
        """
        TODO
        """
        ...


    def place_object(self):
        """
        TODO
        """
        ...

    def move_object(self):
        """
        TODO
        """
        ...



if __name__ == "__main__":

    config_path = Path(__file__).resolve().parents[3] / "config" / "isaacsim_config.yaml"

    isaac = IsaacsimObjectInterface.from_yaml(config_path)
    isaac.start_loop()