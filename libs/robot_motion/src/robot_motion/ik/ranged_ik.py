
import robot_motion.ik.ranged_ik_rust_wrapper as RelaxedIKRust
from robot_motion.ik.ik import IK

class RangedIK(IK):
    """
    Base class for the RangedIK solver.
    Handles the initialization and connection to the underlying Rust library.
    """
    def __init__(self, settings_path:str=None):
        """
        Initialize the Rust-based solver
        Args:
            settings_path (str): Path to setting yaml required for ranged IK
        """
        self.solver = RelaxedIKRust.RelaxedIKRust(settings_path)

