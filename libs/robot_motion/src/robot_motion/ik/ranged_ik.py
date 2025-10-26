# File: base_ranged_ik.py
import numpy as np
import python_wrapper as RelaxedIKRust
import os

# TODO STill need to update function comments

class RangedIK:
    """
    Base class for the RangedIK solver.
    Handles the initialization and connection to the underlying Rust library.
    """
    def __init__(self, settings_path=None):
        print("RangedIK initialized")
        # Initialize the Rust-based solver
        self.solver = RelaxedIKRust.RelaxedIKRust(settings_path)


#  # Below is the original code for testing for palm only
#  #!/usr/bin/env python3

# from ik import IK
# import numpy as np
# import python_wrapper as RelaxedIKRust
# import os

# import rclpy
# from sensor_msgs.msg import JointState

# # TODO: Function comments
# # need to add ccommend ******************************************************************************
# class RangedIK(IK):
#     def __init__(self, settings_path=None):
#         print("RangedIK initialized")
#         # Initialize the Rust-based solver
#         self.solver = RelaxedIKRust.RelaxedIKRust(settings_path)
    
#     def solve(self, x: np.ndarray, base_frame: str, ee_frame: str) -> np.ndarray:
#         """
#         Solve inverse kinematics for a given end-effector target.

#         Parameters
#         ----------
#         x : np.ndarray
#             Target position (and optionally orientation) in world coordinates.
#         base_frame : str
#             Name of the robot's base frame.
#         ee_frame : str
#             Name of the end-effector frame.

#         Returns
#         -------
#         np.ndarray
#             Joint angles computed by RangedIK.
#         """
#         print(f"Solving IK for target {x} from {base_frame} to {ee_frame}")

#         # Convert x to position and orientation arrays
#         # (Assume x contains position + quaternion; adjust if you use a different format)
#         pos = x[:3]
#         quat = x[3:] if len(x) >= 7 else [0, 0, 0, 1]
#         tol = [0.01] * 6  # Example tolerances; adjust as needed

#         # Call the Rust solver
#         result = self.solver.solve_position(pos, quat, tol)

#         # Convert result (C array) to numpy
#         return np.array(result)
    
# def main():
#     """
#     Test the RangedIK solver without ROS.
#     """
#     # Path to your RelaxedIK settings file
#     settings_path = os.path.join(
#         os.path.dirname(__file__),  # folder where ranged_ik.py lives
#         "settings.yaml"
#     )

#     # Initialize the solver
#     rik = RangedIK(settings_path=settings_path)

#     # Define a target pose (x, y, z, qx, qy, qz, qw)
#     target_pose = np.array([0.4, 0.2, 0.3, 0, 1.57, 0, 1])

#     # Solve IK
#     joint_angles = rik.solve(target_pose, base_frame="base_link", ee_frame="ee_link")

#     # Display results
#     print("\n=== IK Solution ===")
#     print("Target Pose:", target_pose)
#     print("Joint Angles:", joint_angles)


# if __name__ == "__main__":
#     main()


# # below is original code for palm and fingers

#!/usr/bin/env python3



# from ik import IK
# import numpy as np
# import python_wrapper as RelaxedIKRust
# import os
# import rclpy
# from sensor_msgs.msg import JointState

# # TODO: Function comments
# # need to add ccommend ******************************************************************************
# class RangedIK(IK):
#     def __init__(self, settings_path=None):
#         print("RangedIK initialized")
#         # Initialize the Rust-based solver
#         self.solver = RelaxedIKRust.RelaxedIKRust(settings_path)

#     def solve(self, x: np.ndarray, base_frame: str, ee_frame: str) -> np.ndarray:
#         """
#         Solve inverse kinematics for a given end-effector target.

#         Parameters
#         ----------
#         x : np.ndarray
#             Target position (and optionally orientation) in world coordinates.
#         base_frame : str
#             Name of the robot's base frame.
#         ee_frame : str
#             Name of the end-effector frame.

#         Returns
#         -------
#         np.ndarray
#             Joint angles computed by RangedIK.

#         """
#         print(f"Solving IK for target {x} from {base_frame} to {ee_frame}")

#         # Convert x to position and orientation arrays
#         # (Assume x contains position + quaternion; adjust if you use a different format)
#         pos = x[:3]
#         quat = x[3:] if len(x) >= 7 else [0, 0, 0, 1]
#         tol = [0.01] * 6  # Example tolerances; adjust as needed

#         # Call the Rust solver
#         result = self.solver.solve_position(pos, quat, tol)

#         # Convert result (C array) to numpy
#         return np.array(result)

#     def solve_multi(self, goals_xyzquat: list) -> np.ndarray:
#         """
#         goals_xyzquat: list of np.array([x,y,z,qx,qy,qz,qw]) – one per chain,
#                        expressed in EACH CHAIN'S BASE FRAME,
#                        in the SAME ORDER as base_links/ee_links in settings.yaml.
#         Returns: concatenated joint angles for all chains.
#         """
#         # Flatten: positions[0:3*N], quats[0:4*N], tols[0:6*N]
#         pos = []
#         quat = []
#         tol = []

#         for g in goals_xyzquat:
#             p = g[:3].tolist()
#             q = g[3:].tolist() if len(g) >= 7 else [0,0,0,1]
#             pos.extend(p)
#             quat.extend(q)
#             tol.extend([0.02, 0.02, 0.02, 3.14, 3.14, 3.14])  # tight pos, loose ori

#         result = self.solver.solve_position(pos, quat, tol)
#         return np.array(result)

   

# def example_wrist_only():

#     """

#     Demonstrates solving IK for the wrist (arm only).

#     Uses a settings file that includes only the arm chain.

#     """

#     print("\n--- Example: Wrist Only ---")



#     # Use a settings_arm.yaml that defines only the 7 DOF arm

#     settings_arm = os.path.join(os.path.dirname(__file__), "settings_arm.yaml")

#     rik = RangedIK(settings_path=settings_arm)



#     # Goal is expressed in the base frame of the arm chain (left_panda_link0)

#     # [x, y, z, qx, qy, qz, qw]

#     wrist_goal = np.array([0.45, 0.00, 0.30, 0, 0, 0, 1])



#     # Solve IK

#     q_arm = rik.solve(wrist_goal, base_frame="left_panda_link0", ee_frame="left_panda_link8")



#     print("Arm Joint Angles (7):", q_arm)





# # ---------------------------------------------------------------------------

# # Example 2 — Wrist and Fingertips

# # ---------------------------------------------------------------------------

# def example_wrist_and_fingertips():

#     """

#     Demonstrates multi-chain IK for wrist + 3 fingertips.

#     Uses the full settings.yaml (arm + fingers).

#     """

#     print("\n--- Example: Wrist + Fingertips ---")



#     # Full YAML (arm + 3 fingers)

#     settings_all = os.path.join(os.path.dirname(__file__), "settings.yaml")

#     rik = RangedIK(settings_path=settings_all)



#     # --- Build target goals (order matches base_links/ee_links in YAML) ---



#     # Chain 0: arm (base = left_panda_link0, ee = left_panda_link8)

#     wrist_goal = np.array([0.45, 0.00, 0.30, 0, 0, 0, 1])



#     # Chains 1–3: fingertips (bases = left_delto_base_link)

#     # For this demo, all fingertips converge on the same “meet point”.

#     meet_point = np.array([0.07, 0.07, 0.7])  # relative to left_delto_base_link

#     f_quat = [0, 0, 0, 1]                      # orientation doesn’t matter here

#     f1_goal = np.concatenate([meet_point - [.03, .01, 0], f_quat])

#     f2_goal = np.concatenate([meet_point- [.01, .03, 0], f_quat])

#     f3_goal = np.concatenate([meet_point- [-.03, -.01, 0], f_quat])



#     # Ordered goals list (one per chain)

#     goals = [wrist_goal, f1_goal, f2_goal, f3_goal]



#     # Solve all at once

#     q_all = rik.solve_multi(goals)



#     # Split the joint vector (7 arm, then 4 per finger)

#     q_arm = q_all[0:7]

#     q_f1 = q_all[7:11]

#     q_f2 = q_all[11:15]

#     q_f3 = q_all[15:19]



#     print(f"Arm Joints (7): {', '.join(map(str, q_arm))}")

#     print(f"Finger 1 (4): {', '.join(map(str, q_f1))}")

#     print(f"Finger 2 (4): {', '.join(map(str, q_f2))}")

#     print(f"Finger 3 (4): {', '.join(map(str, q_f3))}")



# def main():

#     #example_wrist_only()

#     example_wrist_and_fingertips()



# if __name__ == "__main__":

#     main()