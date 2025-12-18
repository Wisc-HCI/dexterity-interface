import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Rotation as R


def pose_to_transformation(pose:np.ndarray) -> np.ndarray:
    """
    Converts a pose to a transformation matrix
    Args:
        pose (np.ndarray): (7,) Pose as [x, y, z, qx, qy, qz, qw] 
            in meters and quaternions
    Returns:
        (np.ndarray): (4,4)  homogeneous transformation matrix. Translation
            in meters and orientation as rotation matrix.
    """
    translation = pose[:3]
    rotation = R.from_quat(pose[3:]).as_matrix()
    T = np.eye(4)
    T[:3, :3] = rotation
    T[:3, 3] = translation
    return T

def transformation_to_pose(T:np.ndarray) -> np.ndarray:
    """
    Converts a transformation matrix to a pose
    Args:
        T (np.ndarray): (4,4) homogeneous transformation matrix. Translation
            in meters and orientation as rotation matrix.
    Returns:
        (np.ndarray): (7,) Pose as [x, y, z, qx, qy, qz, qw] 
            in meters and quaternions
    """
    rot = R.from_matrix(T[:3, :3])
    qx, qy, qz, qw = rot.as_quat()
    x, y, z = T[:3, 3]
    return np.array([x, y, z, qx, qy, qz, qw])


def euler_to_quaternion(euler:np.ndarray) -> np.ndarray:
    """
    Args:
        euler (np.ndarray): (3,) Euler angles as [rx, ry, rz] ([roll, pitch yaw]) 
            in degrees.
    Returns:
        (np.ndarray): (4,) Quaternion as [qx, qy, qz, qw]
    """

    r = R.from_euler('xyz', euler, degrees=True)
    quat = r.as_quat()
    return quat