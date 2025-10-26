
import numpy as np
import os
from ranged_ik import RangedIK

class SingleChainRangedIK(RangedIK):
    """
    A RangedIK solver specialized for single kinematic chains.
    """
    def solve(self, x: np.ndarray, base_frame: str, ee_frame: str) -> np.ndarray:
        """
        Convert a target pose into joint angles using inverse kinematics.

        Args:
            x (np.ndarray): (7,) Target position and orientation in world coordinates
                [x, y, z, qx, qy, qz, qw].
            base_frame (str): Name of the robot's base frame.
            ee_frame (str): Name of the end-effector frame.

        Returns:
            np.ndarray: Joint angles computed by RangedIK.
        """
        print(f"Solving IK for target {x} from {base_frame} to {ee_frame}")

        # Convert x to position and orientation arrays
        # (Assume x contains position + quaternion; adjust if you use a different format)
        pos = x[:3]
        quat = x[3:] if len(x) >= 7 else [0, 0, 0, 1]
        tol = [0.01] * 6  # Example tolerances; adjust as needed

        # Call the Rust solver
        result = self.solver.solve_position(pos, quat, tol)

        # Convert result (C array) to numpy
        return np.array(result)

def example_wrist_only():
    """
        Test the RangedIK solver for solving the wrist 
    """
    # Path to your RelaxedIK settings file
    settings_path = os.path.join(
        os.path.dirname(__file__),  # folder where ranged_ik.py lives
        "settings.yaml"
    )

    # Initialize the solver
    rik = SingleChainRangedIK(settings_path=settings_path)

    # Define a target pose (x, y, z, qx, qy, qz, qw)
    target_pose = np.array([0.4, 0.2, 0.3, 0, 1.57, 0, 1])

    # Solve IK
    joint_angles = rik.solve(target_pose, base_frame="base_link", ee_frame="ee_link")

    # Display results
    print("\n=== IK Solution ===")
    print("Target Pose:", target_pose)
    print("Joint Angles:", joint_angles)

if __name__ == "__main__":
    example_wrist_only()