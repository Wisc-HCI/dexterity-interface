
import robot_motion.ik.ranged_ik_rust_wrapper as RelaxedIKRust
from robot_motion.ik.ik import IK
import os
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
                Note: Any relative paths in the yaml (for the urdf), 
                will resolve to be relative to this package directory (robot_motion).
        """

        pkg_dir =  Path(__file__).resolve().parents[3]
        urdf_root = str(pkg_dir) + os.sep  # Make sure end with "/"

        self.solver = RelaxedIKRust.RelaxedIKRust(settings_path, urdf_root)

