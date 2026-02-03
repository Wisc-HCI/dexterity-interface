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

    
    q_L_arm_joints = q_all[0][0:7]
    q_L_arm_names  = q_all[1][0:7]

    q_R_arm_joints = q_all[0][7:14]
    q_R_arm_names  = q_all[1][7:14]

    # Print in pairs
    print("Left Arm Joint Values:")
    for name, val in zip(q_L_arm_names, q_L_arm_joints):
        print(f"  {name}: {val}")

    print("Right Arm Joint Values:")
    for name, val in zip(q_R_arm_names, q_R_arm_joints):
        print(f"  {name}: {val}")

    rik.reset(np.concatenate([[0,0,0,0,0,0,0], [0,0,0,0,0,0,0]]))  # testings reset function



if __name__ == "__main__":
    bimanual_ik_example()
