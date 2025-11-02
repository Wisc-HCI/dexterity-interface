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
            goals_xyzquat (list): (g, 7) List of np.ndarray([x, y, z, qx, qy, qz, qw]) — one per chain,
                expressed in each chain's base frame, in the same order as
                base_links/ee_links in settings.yaml.

        Returns:
            (np.ndarray): (g, n) Concatenated joint angles for all chains.
            (list[str]): (g, n) Name of each joint
        """
        pos = []
        quat = []
        tol = []

        for g in goals_xyzquat: 
            p = g[:3].tolist()
            q = g[3:].tolist() if len(g) >= 7 else [0,0,0,1]
            pos.extend(p)
            quat.extend(q)
            tol.extend([0.01, 0.01, 0.01, 0.01, 0.01, 0.01])  # keep tolerance low
        print("positions passed to Rust IK:", pos)
        print("quaternions passed to Rust IK:", quat)
        print("tolerances passed to Rust IK:", tol)
        result = self._solver.solve_position(pos, quat, tol)
        result =  np.array(result)
        return result, self._joint_names
