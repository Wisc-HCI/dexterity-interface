from robot_motion.ik.multi_chain_ranged_ik import MultiChainRangedIK
from pathlib import Path
import numpy as np


def bimanual_ik_example():
    """
    Demonstrates multi-chain IK for at palms of bimanual arm system
    """

    settings =  str(Path(__file__).resolve().parents[1] / "ik" / "config" / "bimanual_ik_settings.yaml")

    rik = MultiChainRangedIK(settings_path=settings)

    # Build target goals (order matches base_links/ee_links in YAML) ---
    wrist_goal_left = np.array([-0.2, 0.2, 0.4, 0.707, 0.707, 0, 0])
    wrist_goal_right = np.array([0.2, 0.2, 0.4, 0.707, 0.707, 0, 0])
    
    goals = [wrist_goal_left, wrist_goal_right]
    q_all = rik.solve(goals)

    q_L_arm = q_all[0:7]
    q_R_arm = q_all[7:14]

    print(f"Left Arm Joints (7): {', '.join(map(str, q_L_arm))}")
    print(f"Right Arm Joints (7): {', '.join(map(str, q_R_arm))}")



if __name__ == "__main__":
    bimanual_ik_example()
