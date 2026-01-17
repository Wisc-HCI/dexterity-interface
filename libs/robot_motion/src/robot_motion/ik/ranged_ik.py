
import robot_motion.ik.ranged_ik_rust_wrapper as RelaxedIKRust
from robot_motion.ik.ik import IK
import os
import yaml
from pathlib import Path

class RangedIK(IK):
    """
    Base class for the RangedIK solver.
    Handles the initialization and connection to the underlying Rust library.
    """
    def __init__(self, settings_path:str=None):
        """
        Initialize the Rust-based solver
        Args:
            settings_path (str): Path to setting yaml required for ranged IK. 
                The YAML file should include the following fields:
                - urdf (str): Path to the robot's URDF file. If this is a relative path, it
                will resolve relative to this package directory (robot_motion).
                - link_radius (float): Collision radius (in meters) for each link. Defaults to `0.5`.
                - base_links (list[str]): Names of the base link(s).
                - ee_links (list[str]): Names of the end-effector link(s) corresponding 
                  to each chain to be solved for.  
                - starting_config (list[float]): Flattened list of joint angles 
                  (in radians) representing the initial seed configuration for the solver.  
                - joint_names (list[str]): Ordered list of joint names matching 
                  the indices in `starting_config`.  
        """

        pkg_dir =  Path(__file__).resolve().parents[3]
        urdf_root = str(pkg_dir) + os.sep  # Make sure end with "/"

        self._solver = RelaxedIKRust.RelaxedIKRust(settings_path, urdf_root)

        # Info to return when solving
        with open(settings_path, "r") as f:
            settings = yaml.safe_load(f)

        self._joint_names = settings["joint_names"]
    
    import numpy as np

    def reset(self, joint_state: np.ndarray):
        """
        Reset the internal state of the solver with a new joint_state seed.

        Args:
            joint_state (np.ndarray): Array of joint angles (in radians)
                representing the robot's current joint configuration.
        """
        if not isinstance(joint_state, np.ndarray):
            raise TypeError("joint_state must be a numpy.ndarray")

        self._solver.reset(joint_state.tolist())


            