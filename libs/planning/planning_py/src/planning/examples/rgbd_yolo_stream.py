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
import sys
import time
from pathlib import Path
from typing import Iterable, Literal, TYPE_CHECKING

import cv2
import matplotlib.pyplot as plt
import numpy as np
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


def _load_yaml(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as fh:
        return yaml.safe_load(fh) or {}


def _resolution_from_config(path: Path) -> tuple[int, int]:
    cfg = _load_yaml(path)
    color = cfg.get("color_intrinsics", {})
    return int(color.get("width", 640)), int(color.get("height", 480))


def _label_colors(count: int) -> np.ndarray:
    return plt.cm.tab10(np.linspace(0, 1, max(count, 1)))


def _segmentation_overlay_rgb(rgb: np.ndarray, semantic_mask: np.ndarray, colors: np.ndarray, alpha: float = 0.45):
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


def _project_centroids(
    centroids: np.ndarray,
    T_world_color: np.ndarray,
    color_intrinsics: "CameraIntrinsics",
    image_shape: tuple[int, int],
) -> list[tuple[int, int] | None]:
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


def _update_pointcloud_plot(
    ax,
    point_clouds: np.ndarray,
    centroids: np.ndarray,
    labels: Iterable[str],
    colors: np.ndarray,
    max_points: int = 12000,
):
    ax.cla()
    stacked_points: list[np.ndarray] = []
    for idx, (pc, centroid, label) in enumerate(zip(point_clouds, centroids, labels)):
        if pc is None or pc.size == 0:
            continue
        pc_array = np.asarray(pc, dtype=np.float32)
        if pc_array.shape[0] > max_points:
            selection = _RNG.choice(pc_array.shape[0], size=max_points, replace=False)
            pc_array = pc_array[selection]

        color = colors[idx % len(colors)][:3]
        ax.scatter(pc_array[:, 0], pc_array[:, 1], pc_array[:, 2], s=3, color=color, alpha=0.6, label=label)
        if centroid is not None and np.all(np.isfinite(centroid)):
            ax.scatter(centroid[0], centroid[1], centroid[2], color=color, s=80, marker="*", edgecolors="k")
        stacked_points.append(pc_array)

    if stacked_points:
        stacked = np.concatenate(stacked_points, axis=0)
        mins = stacked.min(axis=0)
        maxs = stacked.max(axis=0)
        ax.set_xlim(mins[0], maxs[0])
        ax.set_ylim(mins[1], maxs[1])
        ax.set_zlim(mins[2], maxs[2])
        ax.legend(loc="upper right")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    ax.set_title("Segmented point cloud (world frame)")
    ax.figure.canvas.draw_idle()


def _init_camera(
    camera_name: Literal["realsense", "kinect"],
    config_path: Path,
    resolution: tuple[int, int],
    fps: int,
    align: str,
    serial: str | None = None,
    device_index: int | None = None,
) -> "RGBDCameraInterface":
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
    parser.add_argument("--fps", type=int, default=15, help="Streaming frame rate.")
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


def main():
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

    plt.ion()
    fig = plt.figure(figsize=(7, 5))
    ax = fig.add_subplot(111, projection="3d")

    try:
        while True:
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
            centroids = yolo.get_centroid(point_clouds)
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

            cv2.imshow("Segmented RGB", rgb_overlay)
            cv2.imshow("Segmented Depth", depth_overlay)

            if len(labels) > 0:
                _update_pointcloud_plot(ax, point_clouds, centroids, labels, colors, max_points=args.max_points)
                plt.pause(0.001)

            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord("q")):
                break
    except KeyboardInterrupt:
        pass
    finally:
        camera.stop()
        cv2.destroyAllWindows()
        plt.close("all")


if __name__ == "__main__":
    main()
