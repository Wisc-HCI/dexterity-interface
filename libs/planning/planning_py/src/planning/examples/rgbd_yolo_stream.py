"""Realtime RGB-D YOLO segmentation using Kinect or RealSense cameras.

Streams RGB + depth, overlays per-object masks on both views, shows segmented
point clouds, and prints centroids for quick accuracy checks on kitchen objects.

Usage (RealSense):
    python -m planning.examples.rgbd_yolo_stream --camera realsense

Usage (Kinect):
    python -m planning.examples.rgbd_yolo_stream --camera kinect --align none
"""

from __future__ import annotations

import argparse
import queue
import sys
import threading
import time
from pathlib import Path
from typing import Iterable, Literal, TYPE_CHECKING

import cv2
import numpy as np
import open3d as o3d
import yaml


if TYPE_CHECKING:  # pragma: no cover - import guards for type checkers
    from sensor_interface.camera.rgbd_camera import CameraIntrinsics, RGBDCameraInterface


_THIS_FILE = Path(__file__).resolve()
_PLANNING_SRC = _THIS_FILE.parents[2]
_PROJECT_ROOT = _THIS_FILE.parents[6]
_SENSOR_SRC = _PROJECT_ROOT / "libs" / "sensor_interface" / "sensor_interface_py" / "src"
for _path in (_PLANNING_SRC, _SENSOR_SRC):
    if str(_path) not in sys.path:
        sys.path.append(str(_path))

from planning.perception.yolo_perception import YoloPerception, _DEFAULT_MODEL_PATH


_SENSOR_CONFIG_DIR = _SENSOR_SRC / "sensor_interface" / "camera" / "config"
_DEFAULT_CONFIGS: dict[str, Path] = {
    "realsense": _SENSOR_CONFIG_DIR / "realsense_config.yaml",
    "kinect": _SENSOR_CONFIG_DIR / "kinect_config.yaml",
}
_RNG = np.random.default_rng(0)

# Matplotlib tab10 colors as a plain array so we don't need matplotlib at all
_TAB10 = np.array([
    [0.122, 0.467, 0.706, 1.0],
    [1.000, 0.498, 0.055, 1.0],
    [0.173, 0.627, 0.173, 1.0],
    [0.839, 0.153, 0.157, 1.0],
    [0.580, 0.404, 0.741, 1.0],
    [0.549, 0.337, 0.294, 1.0],
    [0.890, 0.467, 0.761, 1.0],
    [0.498, 0.498, 0.498, 1.0],
    [0.737, 0.741, 0.133, 1.0],
    [0.090, 0.745, 0.812, 1.0],
], dtype=np.float32)


def _load_yaml(path: Path) -> dict:
    """
    Load a YAML file and return a mapping (empty if the file is blank).

    Args:
        path (Path): Filesystem path to the YAML file.

    Returns:
        dict: Parsed YAML data, or an empty dict when the file is empty.
    """
    with path.open("r", encoding="utf-8") as fh:
        return yaml.safe_load(fh) or {}


def _resolution_from_config(path: Path) -> tuple[int, int]:
    """
    Read color resolution from a camera config YAML with fallback defaults.

    Args:
        path (Path): Filesystem path to the camera config YAML.

    Returns:
        tuple[int, int]: (width, height) for the color stream.
    """
    cfg = _load_yaml(path)
    color = cfg.get("color_intrinsics", {})
    return int(color.get("width", 640)), int(color.get("height", 480))


def _label_colors(count: int) -> np.ndarray:
    """
    Generate a list of distinct colors for labeled overlays.

    Args:
        count (int): Number of labels/classes to colorize.

    Returns:
        np.ndarray: Array of RGBA colors sampled from the tab10 palette.
    """
    idx = np.arange(max(count, 1)) % len(_TAB10)
    return _TAB10[idx]


def _segmentation_overlay_rgb(rgb: np.ndarray, semantic_mask: np.ndarray, colors: np.ndarray, alpha: float = 0.45):
    """
    Blend a class mask onto an RGB image and return BGR for OpenCV display.

    Args:
        rgb (np.ndarray): RGB image as a uint8 array shaped (H, W, 3).
        semantic_mask (np.ndarray): Integer mask with 0 as background and 1..N as labels.
        colors (np.ndarray): RGBA colors for each label.
        alpha (float): Blend factor for the overlay.

    Returns:
        np.ndarray: BGR image suitable for OpenCV visualization.
    """
    rgb_float = rgb.astype(np.float32)
    overlay = np.zeros_like(rgb_float)
    for idx, color in enumerate(colors, start=1):
        mask = semantic_mask == idx
        if not np.any(mask):
            continue
        overlay[mask] = color[:3] * 255.0

    blended = cv2.addWeighted(rgb_float, 1.0, overlay, alpha, 0.0)
    return cv2.cvtColor(blended.astype(np.uint8), cv2.COLOR_RGB2BGR)


def _depth_colormap(depth_m: np.ndarray) -> np.ndarray:
    """
    Convert a depth map in meters into a colored visualization image.

    Args:
        depth_m (np.ndarray): Depth image in meters.

    Returns:
        np.ndarray: Color-mapped depth visualization as uint8 BGR.
    """
    finite = np.isfinite(depth_m)
    if not np.any(finite):
        return np.zeros((*depth_m.shape, 3), dtype=np.uint8)

    finite_vals = depth_m[finite]
    vmin = float(np.nanpercentile(finite_vals, 5))
    vmax = float(np.nanpercentile(finite_vals, 95))
    if not np.isfinite(vmin):
        vmin = 0.0
    if not np.isfinite(vmax) or np.isclose(vmax, vmin):
        vmax = vmin + 1.0

    depth_norm = (np.clip(depth_m, vmin, vmax) - vmin) / (vmax - vmin)
    depth_norm[~finite] = 0.0
    depth_u8 = (depth_norm * 255).astype(np.uint8)
    return cv2.applyColorMap(depth_u8, cv2.COLORMAP_TURBO)


def _depth_overlay(depth_vis: np.ndarray, depth_mask: np.ndarray | None, colors: np.ndarray, alpha: float = 0.45):
    """
    Blend a class mask onto a depth visualization image.

    Args:
        depth_vis (np.ndarray): Color-mapped depth image as uint8.
        depth_mask (np.ndarray | None): Integer mask aligned to depth, or None to skip.
        colors (np.ndarray): RGBA colors for each label.
        alpha (float): Blend factor for the overlay.

    Returns:
        np.ndarray: Depth visualization with overlay applied.
    """
    if depth_mask is None:
        return depth_vis

    overlay = np.zeros_like(depth_vis, dtype=np.float32)
    for idx, color in enumerate(colors, start=1):
        mask = depth_mask == idx
        if not np.any(mask):
            continue
        overlay[mask] = color[:3] * 255.0

    blended = cv2.addWeighted(depth_vis.astype(np.float32), 1.0, overlay, alpha, 0.0)
    return blended.astype(np.uint8)


def _match_display_size(image: np.ndarray, target_shape: tuple[int, int]) -> np.ndarray:
    """
    Resize (and center-crop if needed) an image to match a target (H, W).

    This keeps the target aspect ratio by cropping the larger dimension, which
    effectively "zooms" the source to fill the target size without stretching.
    """
    target_h, target_w = target_shape
    if target_h <= 0 or target_w <= 0:
        return image

    h, w = image.shape[:2]
    if h == target_h and w == target_w:
        return image

    if h == 0 or w == 0:
        return cv2.resize(image, (target_w, target_h), interpolation=cv2.INTER_NEAREST)

    src_aspect = w / h
    tgt_aspect = target_w / target_h
    if not np.isfinite(src_aspect) or not np.isfinite(tgt_aspect):
        return cv2.resize(image, (target_w, target_h), interpolation=cv2.INTER_NEAREST)

    if src_aspect > tgt_aspect:
        new_w = int(round(h * tgt_aspect))
        x0 = max((w - new_w) // 2, 0)
        image = image[:, x0:x0 + new_w]
    elif src_aspect < tgt_aspect:
        new_h = int(round(w / tgt_aspect))
        y0 = max((h - new_h) // 2, 0)
        image = image[y0:y0 + new_h, :]

    return cv2.resize(image, (target_w, target_h), interpolation=cv2.INTER_NEAREST)


def _project_centroids(
    centroids: np.ndarray,
    T_world_color: np.ndarray,
    color_intrinsics: "CameraIntrinsics",
    image_shape: tuple[int, int],
) -> list[tuple[int, int] | None]:
    """
    Project 3D centroids into color image pixels using intrinsics/extrinsics.

    Args:
        centroids (np.ndarray): Array of 3D points in world coordinates.
        T_world_color (np.ndarray): Transform from color frame to world frame.
        color_intrinsics (CameraIntrinsics): Intrinsics for the color camera.
        image_shape (tuple[int, int]): (height, width) of the color image.

    Returns:
        list[tuple[int, int] | None]: Pixel locations for each centroid, or None if invalid.
    """
    T_color_world = np.linalg.inv(T_world_color)
    H, W = image_shape
    pixels: list[tuple[int, int] | None] = []
    for c in centroids:
        if c is None or not np.all(np.isfinite(c)):
            pixels.append(None)
            continue
        pt_world = np.array([c[0], c[1], c[2], 1.0], dtype=np.float32)
        pt_color = T_color_world @ pt_world
        if pt_color[2] <= 0:
            pixels.append(None)
            continue

        u = int(round((pt_color[0] * color_intrinsics.fx) / pt_color[2] + color_intrinsics.cx))
        v = int(round((pt_color[1] * color_intrinsics.fy) / pt_color[2] + color_intrinsics.cy))
        if 0 <= u < W and 0 <= v < H:
            pixels.append((u, v))
        else:
            pixels.append(None)
    return pixels


def _annotate_centroids(
    image: np.ndarray,
    centroids: np.ndarray,
    labels: list[str],
    centroid_pixels: list[tuple[int, int] | None],
    colors: np.ndarray,
):
    """
    Draw centroid text and markers onto an image in-place.

    Args:
        image (np.ndarray): Target image to annotate (modified in-place).
        centroids (np.ndarray): Array of 3D centroid coordinates.
        labels (list[str]): Human-readable labels per centroid.
        centroid_pixels (list[tuple[int, int] | None]): Projected pixel positions per centroid.
        colors (np.ndarray): RGBA colors for each label.
    """
    y = 20
    for idx, (centroid, label, pix) in enumerate(zip(centroids, labels, centroid_pixels)):
        color_rgb = colors[idx % len(colors)][:3]
        color_bgr = tuple(int(255 * c) for c in color_rgb[::-1])

        if centroid is None or not np.all(np.isfinite(centroid)):
            text = f"{label}: centroid unavailable"
        else:
            text = f"{label}: ({centroid[0]:.3f}, {centroid[1]:.3f}, {centroid[2]:.3f})"
        cv2.putText(image, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color_bgr, 1, cv2.LINE_AA)
        y += 18

        if pix is not None:
            cv2.circle(image, pix, 4, color_bgr, -1, lineType=cv2.LINE_AA)


def _update_pointcloud_o3d(
    pcd: "o3d.geometry.PointCloud",
    point_clouds: np.ndarray,
    centroids: np.ndarray,
    labels: Iterable[str],
    colors: np.ndarray,
    max_points: int = 12000,
) -> bool:
    """
    Write updated point cloud data into ``pcd`` in-place.

    Points and per-object colors are written into ``pcd`` in-place so Open3D
    does not need to re-allocate geometry objects every frame.  Centroids are
    appended as a small cluster of white points so they stand out.

    Args:
        pcd: PointCloud geometry to update in-place.
        point_clouds (np.ndarray): Per-object point clouds in world coordinates.
        centroids (np.ndarray): Per-object centroid coordinates.
        labels (Iterable[str]): Labels (unused here, kept for API symmetry).
        colors (np.ndarray): RGBA colors for each label.
        max_points (int): Maximum number of points to show per object.

    Returns:
        bool: True if ``pcd`` now contains at least one point, False otherwise.
    """
    all_pts: list[np.ndarray] = []
    all_col: list[np.ndarray] = []

    for idx, (pc, centroid) in enumerate(zip(point_clouds, centroids)):
        if pc is None or len(pc) == 0:
            continue
        pc_array = np.asarray(pc, dtype=np.float64)
        if pc_array.shape[0] > max_points:
            sel = _RNG.choice(pc_array.shape[0], size=max_points, replace=False)
            pc_array = pc_array[sel]

        color = colors[idx % len(colors)][:3].astype(np.float64)
        all_pts.append(pc_array)
        all_col.append(np.tile(color, (len(pc_array), 1)))

        # Mark centroid with a small black cluster so it's easy to spot
        if centroid is not None and np.all(np.isfinite(centroid)):
            all_pts.append(np.tile(centroid, (6, 1)).astype(np.float64))
            all_col.append(np.zeros((6, 3), dtype=np.float64))

    if not all_pts:
        return False

    pcd.points = o3d.utility.Vector3dVector(np.concatenate(all_pts, axis=0))
    pcd.colors = o3d.utility.Vector3dVector(np.concatenate(all_col, axis=0))
    return True


def _init_camera(
    camera_name: Literal["realsense", "kinect"],
    config_path: Path,
    resolution: tuple[int, int],
    fps: int,
    align: str,
    serial: str | None = None,
    device_index: int | None = None,
) -> "RGBDCameraInterface":
    """
    Create and start the selected RGB-D camera interface.

    Args:
        camera_name (Literal["realsense", "kinect"]): Camera backend to initialize.
        config_path (Path): Path to the camera YAML configuration.
        resolution (tuple[int, int]): (width, height) to stream.
        fps (int): Target frames per second.
        align (str): Frame alignment mode passed to the interface.
        serial (str | None): RealSense serial number, if required.
        device_index (int | None): Kinect device index, if required.

    Returns:
        RGBDCameraInterface: Started camera interface instance.
    """
    if camera_name == "realsense":
        from sensor_interface.camera.realsense_interface import RealsenseInterface

        camera = RealsenseInterface.from_yaml(str(config_path))
        camera.start(resolution=resolution, fps=fps, align=align, serial=serial)
        return camera

    from sensor_interface.camera.kinect_interface import KinectInterface

    camera = KinectInterface.from_yaml(str(config_path))
    camera.start(resolution=resolution, fps=fps, align=align, device=device_index)
    return camera


def parse_args() -> argparse.Namespace:
    """
    Parse CLI arguments for the RGB-D YOLO streaming demo.

    Returns:
        argparse.Namespace: Parsed command-line arguments.
    """
    parser = argparse.ArgumentParser(description="Realtime RGB-D YOLO stream with point clouds and centroids.")
    parser.add_argument(
        "--camera",
        choices=["realsense", "kinect"],
        default="realsense",
        help="Which RGB-D interface to use.",
    )
    parser.add_argument(
        "--camera-config",
        type=Path,
        help="Path to the camera YAML. Defaults to the packaged config for the selected camera.",
    )
    parser.add_argument(
        "--align",
        choices=["none", "color", "depth"],
        default="none",
        help="Frame alignment strategy passed to the camera interface.",
    )
    parser.add_argument("--fps", type=int, default=30, help="Streaming frame rate.")
    parser.add_argument("--serial", type=str, help="Camera serial (RealSense only).")
    parser.add_argument("--device-index", type=int, help="Device index for Kinect.")
    parser.add_argument(
        "--model",
        type=Path,
        default=_DEFAULT_MODEL_PATH,
        help="YOLOv11 segmentation checkpoint to load.",
    )
    parser.add_argument("--confidence", type=float, default=0.3, help="YOLO detection confidence threshold.")
    parser.add_argument("--iou", type=float, default=0.45, help="YOLO IoU threshold.")
    parser.add_argument("--device", type=str, default=None, help="Torch device string, e.g., cuda, cpu, or mps.")
    parser.add_argument(
        "--classes",
        type=int,
        nargs="+",
        help="Optional list of class IDs to keep (see YOLO model labels).",
    )
    parser.add_argument(
        "--transform-config",
        type=Path,
        help="Optional YAML containing T_world_color to express point clouds in world frame.",
    )
    parser.add_argument(
        "--max-points",
        type=int,
        default=12000,
        help="Maximum points per object to visualize in the live point cloud window.",
    )
    return parser.parse_args()


def _inference_worker(
    camera,
    yolo,
    result_queue: queue.Queue,
    stop_event: threading.Event,
    max_points: int,
):
    """
    Background thread: capture frames, run YOLO inference, and push results.

    Results are placed into ``result_queue`` (capacity 1) so the main thread
    always gets the most recent frame without accumulating a backlog.

    Args:
        camera: Started RGB-D camera interface.
        yolo: YoloPerception instance.
        result_queue (queue.Queue): Queue shared with the main thread.
        stop_event (threading.Event): Set this to request a clean shutdown.
        max_points (int): Max points per object forwarded to the plot helper.
    """
    while not stop_event.is_set():
        try:
            frame = camera.latest()
        except RuntimeError:
            time.sleep(0.01)
            continue

        if frame.color is None or frame.depth is None:
            time.sleep(0.01)
            continue

        start = time.time()
        rgb = frame.color
        depth_m = frame.depth

        semantic_mask, labels = yolo.detect_rgb(rgb)
        point_clouds, labels, depth_mask = yolo.get_object_point_clouds(
            depth_m, semantic_mask, labels, return_depth_mask=True
        )
        point_clouds = yolo.filter_point_clouds(point_clouds)
        centroids = yolo.get_centroid(point_clouds, method="bbox")
        colors = _label_colors(len(labels))

        centroid_pixels = _project_centroids(
            centroids,
            yolo.T_world_color,
            camera.color_intrinsics,
            (rgb.shape[0], rgb.shape[1]),
        )

        rgb_overlay = _segmentation_overlay_rgb(rgb, semantic_mask, colors)
        depth_vis = _depth_colormap(depth_m)
        depth_overlay = _depth_overlay(depth_vis, depth_mask, colors)
        depth_overlay = _match_display_size(depth_overlay, rgb_overlay.shape[:2])

        _annotate_centroids(rgb_overlay, centroids, labels, centroid_pixels, colors)
        _annotate_centroids(depth_overlay, centroids, labels, centroid_pixels, colors)

        fps = 1.0 / max(time.time() - start, 1e-6)
        cv2.putText(
            rgb_overlay,
            f"FPS: {fps:.1f}",
            (10, rgb_overlay.shape[0] - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )

        # Discard a stale result so the queue never fills up
        try:
            result_queue.get_nowait()
        except queue.Empty:
            pass
        result_queue.put((rgb_overlay, depth_overlay, point_clouds, centroids, labels, colors))


def main():
    """
    Run the RGB-D streaming loop with YOLO segmentation and visualization.

    YOLO inference runs on a background thread.  The main thread owns both the
    Open3D window and the OpenCV
    windows, polling the inference queue each iteration.

    Returns:
        None
    """
    args = parse_args()

    camera_cfg = args.camera_config
    if camera_cfg is None:
        camera_cfg = _DEFAULT_CONFIGS.get(args.camera)
    if camera_cfg is None:
        raise FileNotFoundError(f"No default config found for camera {args.camera}")
    camera_cfg = camera_cfg.expanduser().resolve()
    if not camera_cfg.exists():
        raise FileNotFoundError(f"Camera config not found: {camera_cfg}")

    resolution = _resolution_from_config(camera_cfg)
    print(f"Starting {args.camera} with resolution {resolution}, fps={args.fps}, align={args.align}")
    camera = _init_camera(
        args.camera,
        camera_cfg,
        resolution=resolution,
        fps=args.fps,
        align=args.align,
        serial=args.serial,
        device_index=args.device_index,
    )

    transform_kwargs = YoloPerception.load_constructor_kwargs(args.transform_config)
    yolo = YoloPerception(
        camera,
        model_path=args.model,
        conf=args.confidence,
        iou=args.iou,
        classes=args.classes,
        device=args.device,
        **transform_kwargs,
    )

    # Open3D visualizer — OpenGL-backed, rotation is handled by its own event loop.
    # The geometry is added lazily (on the first frame with actual points) to avoid
    # the "0 points when creating axis-aligned bounding box" warning.
    vis = o3d.visualization.Visualizer()
    vis.create_window("Segmented Point Cloud (world frame)", width=700, height=500)
    pcd = o3d.geometry.PointCloud()
    pcd_in_scene = False  # becomes True once we have real data

    result_queue: queue.Queue = queue.Queue(maxsize=1)
    stop_event = threading.Event()
    worker = threading.Thread(
        target=_inference_worker,
        args=(camera, yolo, result_queue, stop_event, args.max_points),
        daemon=True,
    )
    worker.start()

    try:
        while True:
            # Pull the latest inference result if one is ready (non-blocking)
            try:
                rgb_overlay, depth_overlay, point_clouds, centroids, labels, colors = result_queue.get_nowait()
                cv2.imshow("Segmented RGB", rgb_overlay)
                cv2.imshow("Segmented Depth", depth_overlay)
                if len(labels) > 0:
                    has_pts = _update_pointcloud_o3d(
                        pcd, point_clouds, centroids, labels, colors, max_points=args.max_points
                    )
                    if has_pts:
                        if not pcd_in_scene:
                            vis.add_geometry(pcd)
                            vis.reset_view_point(True)
                            pcd_in_scene = True
                        else:
                            vis.update_geometry(pcd)
            except queue.Empty:
                pass

            # Drive the Open3D event loop every iteration — keeps rotation smooth
            if not vis.poll_events():
                break  # user closed the Open3D window
            vis.update_renderer()

            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord("q")):
                break
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        worker.join(timeout=2.0)
        camera.stop()
        cv2.destroyAllWindows()
        vis.destroy_window()


if __name__ == "__main__":
    main()
