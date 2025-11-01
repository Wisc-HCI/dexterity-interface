# File: multi_chain_ik.py
import numpy as np
import os
from robot_motion.ik.ranged_ik import RangedIK

class MultiChainRangedIK(RangedIK):
    """
    A RangedIK solver specialized for multiple kinematic chains.
    """
    def solve(self, goals_xyzquat: list) -> np.ndarray:
        """
        Solve inverse kinematics for multiple end-effectors (chains).

        Args:
            goals_xyzquat (list): List of np.ndarray([x, y, z, qx, qy, qz, qw]) — one per chain,
                expressed in each chain’s base frame, in the same order as
                base_links/ee_links in settings.yaml.

        Returns:
            np.ndarray: Concatenated joint angles for all chains.
        """
        pos = []
        quat = []
        tol = []

        #TODO: maybe separate the wrist and finger for different solves.
        for g in goals_xyzquat: 
            p = g[:3].tolist()
            q = g[3:].tolist() if len(g) >= 7 else [0,0,0,1]
            pos.extend(p)
            quat.extend(q)
            tol.extend([0.01, 0.01, 0.01, 0.01, 0.01, 0.01])  # keep tolerance low
        print("positions passed to Rust IK:", pos)
        print("quaternions passed to Rust IK:", quat)
        print("tolerances passed to Rust IK:", tol)
        result = self.solver.solve_position(pos, quat, tol)

        return np.array(result)

def example_wrist_and_fingertips():

    """

    Demonstrates multi-chain IK for wrist + 3 fingertips.

    Uses the full settings.yaml (arm + fingers).

    """

    print("\n--- Example: Wrist + Fingertips ---")

    # Full YAML (arm + 3 fingers)
    settings_all = os.path.join(os.path.dirname(__file__), "settings.yaml")
    rik = MultiChainRangedIK(settings_path=settings_all)

    # --- Build target goals (order matches base_links/ee_links in YAML) ---

    # Chain 0: arm (base = left_panda_link0, ee = left_panda_link8)

    wrist_goal = np.array([0.45, 0.00, 0.30, 0, 1.57, 0, 1])

    # Chains 1–3: fingertips (bases = left_delto_base_link)

    # For this demo, all fingertips converge on the same “meet point”.
    meet_point = np.array([0.07, 0.07, 0.7])  # relative to left_delto_base_link

    f_quat = [0, 0, 0, 1]                      # orientation doesn’t matter here
    f1_goal = np.concatenate([meet_point - [.03, .01, 0], f_quat])
    f2_goal = np.concatenate([meet_point- [.01, .03, 0], f_quat])
    f3_goal = np.concatenate([meet_point- [-.03, -.01, 0], f_quat])

    # Ordered goals list (one per chain)
    goals = [wrist_goal, f1_goal, f2_goal, f3_goal]

    # Solve all at once
    q_all = rik.solve(goals)

    # Split the joint vector (7 arm, then 4 per finger)
    q_arm = q_all[0:7]
    q_f1 = q_all[7:11]
    q_f2 = q_all[11:15]
    q_f3 = q_all[15:19]

    print(f"Arm Joints (7): {', '.join(map(str, q_arm))}")
    print(f"Finger 1 (4): {', '.join(map(str, q_f1))}")
    print(f"Finger 2 (4): {', '.join(map(str, q_f2))}")
    print(f"Finger 3 (4): {', '.join(map(str, q_f3))}")

def example_2_wrists():

    """

    Demonstrates multi-chain IK for wrist + 3 fingertips.

    Uses the full settings.yaml (arm + fingers).

    """

    print("\n--- Example: 2 Wrists ---")

    # Full YAML (arm + 3 fingers)
    settings_all = os.path.join(os.path.dirname(__file__), "settings.yaml")
    rik = MultiChainRangedIK(settings_path=settings_all)

    # --- Build target goals (order matches base_links/ee_links in YAML) ---

    wrist_goal_left = np.array([0, .2, .5, 0, 1.57, 0, 1])

    wrist_goal_right = np.array([0, .2, .5, 0, -1.57, 0, 1])
    

    # Ordered goals list (one per chain)
    goals = [wrist_goal_left, wrist_goal_right]

    # Solve all at once
    q_all = rik.solve(goals)

    # Split the joint vector (7 arm, then 4 per finger)
    # print("q_all length", len(q_all))
    # print("q_all", q_all)
    q_L_arm = q_all[0:7]
    q_R_arm = q_all[7:14]

    print(f"Left Arm Joints (7): {', '.join(map(str, q_L_arm))}")
    print(f"Right Arm Joints (7): {', '.join(map(str, q_R_arm))}")

if __name__ == "__main__":
    # example_wrist_and_fingertips()
    example_2_wrists()