import argparse
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import yaml

ARUCO_DICT   = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
ARUCO_PARAMS = cv2.aruco.DetectorParameters()
CONFIG_DIR   = Path(__file__).resolve().parent


def parse_args():
    """
    Parse command-line arguments.
    """
    p = argparse.ArgumentParser()
    p.add_argument("--config",      required=True,  help="Path to camera YAML config.")
    p.add_argument("--marker-size", type=float, required=True, help="Marker side length in metres.")
    p.add_argument("--marker-pos",  type=float, nargs=3, default=[0.0, 0.0, 0.0], metavar=("X", "Y", "Z"),
                        help="World-frame position of the marker centre in metres (default: 0 0 0).")
    p.add_argument("--frames",      type=int, default=30, help="Frames to average (default: 30).")
    p.add_argument("--write",       action="store_true", help="Write result to table_world_transform.yaml.")
    return p.parse_args()


def init_camera(config_path: Path) -> "RGBDCameraInterface":
    """
    Construct and start the appropriate camera from a config YAML file.
    Camera type (RealSense or Kinect) is inferred from the metadata.device field.

    Args:
        config_path (Path): Path to the camera YAML config file.

    Returns:
        RGBDCameraInterface: Started camera interface ready to stream.
    """
    with open(config_path) as f:
        config = yaml.safe_load(f)

    device = config.get("metadata", {}).get("device", "").lower()

    if "realsense" in device:
        from sensor_interface.camera.realsense_interface import RealsenseInterface
        cam = RealsenseInterface.from_yaml(str(config_path))
    elif "kinect" in device:
        from sensor_interface.camera.kinect_interface import KinectInterface
        cam = KinectInterface.from_yaml(str(config_path))
    else:
        print("[ERROR] Unknown camera type in config metadata.", file=sys.stderr)
        raise SystemExit(1)

    resolution = (cam.color_intrinsics.width, cam.color_intrinsics.height)
    cam.start(resolution=resolution, align="color")

    # Wait for first valid frame
    for _ in range(50):
        try:
            cam.latest()
            break
        except RuntimeError:
            time.sleep(0.05)

    return cam


def detect(gray: np.ndarray) -> np.ndarray:
    """
    Detect a single DICT_4X4_50 ArUco marker in a grayscale image.

    Args:
        gray (np.ndarray): (H, W) uint8 grayscale image.

    Returns:
        (np.ndarray | None): (4, 2) corner pixel coordinates, or None if not found.
    """
    corners, ids, _ = cv2.aruco.ArucoDetector(ARUCO_DICT, ARUCO_PARAMS).detectMarkers(gray)
    if ids is None or len(ids) == 0:
        return None
    return corners[0].reshape(4, 2)


def solve(corners: np.ndarray, marker_size: float, K: np.ndarray, dist: np.ndarray) -> np.ndarray:
    """
    Solve for the marker pose in the camera frame using PnP.

    Args:
        corners (np.ndarray): (4, 2) detected corner pixel coordinates.
        marker_size (float): Physical side length of the marker in metres.
        K (np.ndarray): (3, 3) camera intrinsic matrix.
        dist (np.ndarray): (5,) distortion coefficients.

    Returns:
        (np.ndarray | None): (4, 4) T_color_marker, or None on failure.
    """
    h = marker_size / 2.0
    # Corners in marker frame: top-left, top-right, bottom-right, bottom-left
    obj_pts = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], dtype=np.float64)
    flag = getattr(cv2, "SOLVEPNP_IPPE_SQUARE", cv2.SOLVEPNP_ITERATIVE)
    ok, rvec, tvec = cv2.solvePnP(obj_pts, corners, K, dist, flags=flag)
    if not ok:
        return None
    R, _ = cv2.Rodrigues(rvec)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3,  3] = tvec.reshape(3)
    return T


def average(transforms: list[np.ndarray]) -> np.ndarray:
    """
    Average a list of 4x4 rigid transforms.
    Rotations are averaged via SVD of the summed rotation matrices.
    Translations are averaged arithmetically.

    Args:
        transforms (list[np.ndarray]): List of (4, 4) homogeneous transforms.

    Returns:
        (np.ndarray): (4, 4) averaged transform.
    """
    Rs = np.stack([T[:3, :3] for T in transforms])
    ts = np.stack([T[:3,  3] for T in transforms])
    U, _, Vt = np.linalg.svd(Rs.sum(axis=0))
    R = U @ Vt
    if np.linalg.det(R) < 0:  # Correct reflection
        U[:, -1] *= -1
        R = U @ Vt
    T = np.eye(4)
    T[:3, :3] = R
    T[:3,  3] = ts.mean(axis=0)
    return T


def main():
    """
    Collect color frames, detect the ArUco marker, and compute T_world_color
    by averaging pose estimates across --frames detections.
    Optionally writes the result to table_world_transform.yaml with --write.
    """
    args = parse_args()

    config_path = Path(args.config).resolve()
    cam = init_camera(config_path)

    intr = cam.color_intrinsics
    K    = np.array([[intr.fx, 0, intr.cx], [0, intr.fy, intr.cy], [0, 0, 1]], dtype=np.float64)
    dist = np.array(intr.distortion if intr.distortion is not None else [0.0] * 5, dtype=np.float64)

    transforms: list[np.ndarray] = []
    T_world_marker = np.eye(4)
    T_world_marker[:3, 3] = args.marker_pos

    print(f"[INFO] Collecting {args.frames} frames — place marker flat at {args.marker_pos}...")

    try:
        while len(transforms) < args.frames:
            try:
                frame = cam.latest()
            except RuntimeError:
                time.sleep(0.01)
                continue

            gray = cv2.cvtColor(frame.color, cv2.COLOR_RGB2GRAY)
            corners = detect(gray)
            if corners is None:
                continue

            T_color_marker = solve(corners, args.marker_size, K, dist)
            if T_color_marker is None:
                continue

            # T_world_color = T_world_marker @ T_marker_color
            transforms.append(T_world_marker @ np.linalg.inv(T_color_marker))
            print(f"  [{len(transforms)}/{args.frames}]", end="\r", flush=True)
    finally:
        cam.stop()

    print()
    if not transforms:
        print("[ERROR] No detections — check marker is visible and DICT_4X4_50 is used.", file=sys.stderr)
        raise SystemExit(1)

    T_world_color = average(transforms)
    T_world_color_list = [[round(v, 9) for v in row] for row in T_world_color.tolist()]

    print("T_world_color:")
    for row in T_world_color_list:
        print(" ", row)

    if args.write:
        out_path = config_path.parent / "table_world_transform.yaml"
  
        with open(out_path) as f:
            out = yaml.safe_load(f) or {}
  
        out["T_world_color"] = T_world_color_list
        with open(out_path, "w") as f:
            yaml.dump(out, f, default_flow_style=False, sort_keys=False)
        print(f"[INFO] Written to {out_path}")


if __name__ == "__main__":
    main()
