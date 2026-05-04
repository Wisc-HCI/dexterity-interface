import numpy as np
from scipy.spatial.transform import Rotation, Slerp


def trapezoidal_path_param(n_steps: int, t_ramp_frac: float) -> np.ndarray:
    """
    Returns path parameter s in [0, 1] for n_steps+1 uniformly-spaced time samples
    using a trapezoidal velocity profile (ramp up, cruise, ramp down).

    Args:
        n_steps (int): Number of trajectory steps.
        t_ramp_frac (float): Fraction of total time spent ramping up (and separately,
            ramping down). Clamped to [0, 0.5]. At 0.5 there is no cruise phase
            (pure triangle); at 0 it degenerates to constant velocity.
    Returns:
        np.ndarray: (n_steps+1,) path parameters in [0, 1].

    Source: Claude
    """
    t = np.linspace(0, 1, n_steps + 1)
    b = np.clip(t_ramp_frac, 0.0, 0.5)

    if b < 1e-6:
        return t

    # Peak velocity normalized so the total path integral equals 1
    v_peak = 1.0 / (1.0 - b)

    s = np.where(
        t <= b,
        0.5 * (v_peak / b) * t ** 2,
        np.where(
            t <= 1.0 - b,
            0.5 * v_peak * b + v_peak * (t - b),
            1.0 - 0.5 * (v_peak / b) * (1.0 - t) ** 2,
        ),
    )
    return s


def interpolate_cartesian_trajectory(start_pose: np.ndarray, goal_pose: np.ndarray, dt: float,
                                     velocity: float, angular_velocity: float,
                                     acceleration: float = 0) -> np.ndarray:
    """
    Generate a Cartesian trajectory from start pose to goal pose using
    linear interpolation for position and SLERP for orientation. Duration is determined by whichever
    motion takes longer (linear or rotational) so both complete at the same time. This means
    the actual velocity may be lower.

    Args:
        start_pose (np.ndarray): (7,) Start pose [x, y, z, qx, qy, qz, qw] in m/rad.
        goal_pose (np.ndarray): (7,) Target pose [x, y, z, qx, qy, qz, qw] in m/rad.
        dt (float): Time step between trajectory points in seconds.
        velocity (float): Desired linear velocity in m/s.
        angular_velocity (float): Desired angular velocity in rad/s.
        acceleration (float): Uses a trapezoidal velocity profile that
            ramps up and down at this rate (m/s^2). The ramp time is velocity / acceleration,
            capped at half the total duration. If 0, uses constant velocity.
    Returns:
        np.ndarray: (N, 7) Array of interpolated poses [x, y, z, qx, qy, qz, qw].
    """
    start_pos = start_pose[:3]
    goal_pos = goal_pose[:3]

    distance = np.linalg.norm(goal_pos - start_pos)

    start_rot = Rotation.from_quat(start_pose[3:])
    goal_rot = Rotation.from_quat(goal_pose[3:])
    angle = (goal_rot * start_rot.inv()).magnitude()

    total_time = max(distance / velocity, angle / angular_velocity)

    if total_time < 1e-6:
        return goal_pose.reshape(1, 7)

    n_steps = max(1, int(np.ceil(total_time / dt)))

    if acceleration > 0:
        t_ramp = min(velocity / acceleration, total_time / 2.0)
        s = trapezoidal_path_param(n_steps, t_ramp / total_time)
    else:
        s = np.linspace(0, 1, n_steps + 1)

    # Linear interpolation for position
    positions = start_pos + np.outer(s, goal_pos - start_pos)

    # SLERP for orientation (scipy uses [qx, qy, qz, qw] scalar-last)
    start_quat = start_pose[3:]
    goal_quat = goal_pose[3:]

    rotations = Rotation.from_quat(np.array([start_quat, goal_quat]))
    slerp = Slerp([0, 1], rotations)
    orientations = slerp(s).as_quat()

    return np.hstack([positions, orientations])
