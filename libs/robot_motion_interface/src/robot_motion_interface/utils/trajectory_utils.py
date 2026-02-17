import numpy as np
from scipy.spatial.transform import Rotation, Slerp


def interpolate_cartesian_trajectory(start_pose: np.ndarray, goal_pose: np.ndarray,
                                     dt: float, velocity: float) -> np.ndarray:
    """
    Generate a Cartesian trajectory from start pose to goal pose using
    linear interpolation for position and SLERP for orientation.

    Args:
        start_pose (np.ndarray): (7,) Start pose [x, y, z, qx, qy, qz, qw] in m/rad.
        goal_pose (np.ndarray): (7,) Target pose [x, y, z, qx, qy, qz, qw] in m/rad.
        dt (float): Time step between trajectory points in seconds.
        velocity (float): Desired linear velocity in m/s.

    Returns:
        np.ndarray: (N, 7) Array of interpolated poses [x, y, z, qx, qy, qz, qw].
    """
    start_pos = start_pose[:3]
    goal_pos = goal_pose[:3]

    distance = np.linalg.norm(goal_pos - start_pos)
    if distance < 1e-6:
        return goal_pose.reshape(1, 7)

    total_time = distance / velocity
    n_steps = max(1, int(np.ceil(total_time / dt)))

    t = np.linspace(0, 1, n_steps + 1)

    # Linear interpolation for position
    positions = start_pos + np.outer(t, goal_pos - start_pos)

    # SLERP for orientation (scipy uses [qx, qy, qz, qw] scalar-last)
    start_quat = start_pose[3:]
    goal_quat = goal_pose[3:]

    rotations = Rotation.from_quat(np.array([start_quat, goal_quat]))
    slerp = Slerp([0, 1], rotations)
    orientations = slerp(t).as_quat()

    return np.hstack([positions, orientations])
