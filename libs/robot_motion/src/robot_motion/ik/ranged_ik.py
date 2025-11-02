
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
                If a relative path is given, it will resolve to be relative to the package directory (robot_motion) directory.
        """

        pkg_dir =  Path(__file__).resolve().parents[3]
        settings_path = str((pkg_dir / settings_path).resolve())
        urdf_root = str(pkg_dir) + os.sep  # Make sure end with "/"

        self.solver = RelaxedIKRust.RelaxedIKRust(settings_path, urdf_root)

