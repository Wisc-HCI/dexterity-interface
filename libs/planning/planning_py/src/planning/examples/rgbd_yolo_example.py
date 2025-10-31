"""Example script that runs YOLOv11 segmentation on an RGB-D image pair
and visualizes per-object point clouds in the world frame.

Usage:
    python -m planning.examples.rgbd_yolo_example \
        --config libs/planning/planning_py/src/planning/config/kitchen_example.yaml \
        --output pointcloud.png
"""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Tuple

import sys


_THIS_FILE = Path(__file__).resolve()
_PLANNING_SRC = _THIS_FILE.parents[2]
_PROJECT_ROOT = _THIS_FILE.parents[6]
_SENSOR_SRC = _PROJECT_ROOT / "libs" / "sensor_interface" / "sensor_interface_py" / "src"
for _path in (_PLANNING_SRC, _SENSOR_SRC):
    if str(_path) not in sys.path:
        sys.path.append(str(_path))

import matplotlib.pyplot as plt
import numpy as np
import yaml
from PIL import Image

from planning.perception.yolo_perception import YoloPerception
from sensor_interface.camera.rgbd_camera import CameraIntrinsics, RGBDCameraInterface


class StaticRGBDCamera(RGBDCameraInterface):
    """Minimal RGB-D camera implementation that just exposes intrinsics."""

    def __init__(self, color_intrinsics: CameraIntrinsics, depth_intrinsics: CameraIntrinsics, T_color_depth: np.ndarray):
        """
        Store the camera parameters for replaying static RGB-D data.

        Args:
            color_intrinsics (CameraIntrinsics): Intrinsics describing the color imager.
            depth_intrinsics (CameraIntrinsics): Intrinsics describing the depth imager.
            T_color_depth (np.ndarray): (4, 4) homogeneous transform mapping depth points into the
                color optical frame.
        """
        super().__init__(color_intrinsics, depth_intrinsics, T_color_depth)

    def start(self, *args, **kwargs):  # pragma: no cover - example only
        """Static camera does not support streaming sessions."""
        raise NotImplementedError("Static camera does not stream frames")

    def stop(self):  # pragma: no cover - example only
        """Static camera does not support streaming sessions."""
        raise NotImplementedError("Static camera does not stream frames")

    def is_running(self) -> bool:  # pragma: no cover - example only
        """Static camera never streams, so this always returns False."""
        return False

    def latest(self):  # pragma: no cover - example only
        """Static camera cannot provide live frames."""
        raise NotImplementedError("Static camera does not stream frames")


def _load_config(path: Path) -> dict:
    """
    Load a YAML configuration file from disk.

    Args:
        path (Path): Filesystem path to the YAML configuration file.

    Returns:
        dict: Parsed YAML contents with empty files defaulting to an empty dictionary.
    """
    with path.open("r", encoding="utf-8") as fh:
        return yaml.safe_load(fh) or {}


def _build_camera(camera_cfg: dict, base_dir: Path) -> Tuple[StaticRGBDCamera, Path | None]:
    """
    Construct a static RGB-D camera from a parsed configuration dictionary.

    Args:
        camera_cfg (dict): Calibration parameters loaded from the configuration file.
        base_dir (Path): Directory used to resolve relative resource paths.

    Returns:
        tuple[StaticRGBDCamera, Path | None]: Initialized camera and optional transform config path.
    """
    color = camera_cfg["color_intrinsics"]
    depth = camera_cfg["depth_intrinsics"]

    color_intr = CameraIntrinsics(
        width=int(color["width"]),
        height=int(color["height"]),
        fx=float(color["fx"]),
        fy=float(color["fy"]),
        cx=float(color["cx"]),
        cy=float(color["cy"]),
        distortion=None,
    )
    depth_intr = CameraIntrinsics(
        width=int(depth["width"]),
        height=int(depth["height"]),
        fx=float(depth["fx"]),
        fy=float(depth["fy"]),
        cx=float(depth["cx"]),
        cy=float(depth["cy"]),
        distortion=None,
    )

    T_color_depth = np.array(camera_cfg.get("T_color_depth", np.eye(4)), dtype=np.float32)

    transform_path = camera_cfg.get("transform_config")
    if transform_path is not None:
        transform_path = (base_dir / transform_path).resolve()

    camera = StaticRGBDCamera(color_intr, depth_intr, T_color_depth)
    return camera, transform_path


def _load_rgb(path: Path) -> np.ndarray:
    """
    Load an RGB image as a NumPy array.

    Args:
        path (Path): Path to the RGB image file.

    Returns:
        np.ndarray: Array of shape (H, W, 3) in RGB channel order.
    """
    return np.array(Image.open(path).convert("RGB"))


def _load_depth(path: Path) -> np.ndarray:
    """
    Load the raw depth image preserving the sensor's native precision.

    Args:
        path (Path): Path to the depth image captured from the sensor.

    Returns:
        np.ndarray: Array of raw disparity or depth values as float32.
    """
    return np.array(Image.open(path), dtype=np.float32)


def _convert_depth(raw_depth: np.ndarray, cfg: dict) -> np.ndarray:
    """
    Convert raw depth/disparity values into meters using calibration parameters.

    Args:
        raw_depth (np.ndarray): Raw depth or disparity image as returned by `_load_depth`.
        cfg (dict): Dictionary containing calibration entries `param1`, `param2`, and optional
            `scale_divisor` / `max_depth_m` fields.

    Returns:
        np.ndarray: Depth image in meters with invalid entries set to NaN.
    """
    scale_divisor = float(cfg.get("scale_divisor", 1.0))
    param1 = float(cfg["param1"])
    param2 = float(cfg["param2"])

    depth_m = param1 / (param2 + raw_depth / scale_divisor)
    depth_m = depth_m.astype(np.float32)

    max_depth = cfg.get("max_depth_m")
    if max_depth is not None:
        max_depth = float(max_depth)
        invalid = (depth_m <= 0) | (depth_m > max_depth)
        depth_m[invalid] = np.nan
    else:
        depth_m[depth_m <= 0] = np.nan

    return depth_m


def _label_colors(label_count: int) -> np.ndarray:
    """
    Generate consistent RGBA colors for segmentation overlays and plots.

    Args:
        label_count (int): Number of unique object labels present in the scene.

    Returns:
        np.ndarray: Array of RGBA colors sized (label_count, 4).
    """
    return plt.cm.tab10(np.linspace(0, 1, max(label_count, 1)))


def _reorient_for_camera_frame(points: np.ndarray) -> np.ndarray:
    """
    Reorder axes so plots follow the OpenCV camera convention (Z forward, X right, Y down).

    Args:
        points (np.ndarray): Array of 3D points expressed in the world frame.

    Returns:
        np.ndarray: Reordered points aligned with the camera visualization convention.
    """
    pts = np.asarray(points, dtype=np.float32)
    if pts.size == 0:
        return pts.reshape(-1, 3)

    oriented = np.empty_like(pts, dtype=np.float32)
    oriented[:, 0] = pts[:, 0]  # X right
    oriented[:, 1] = pts[:, 2]  # Z forward
    oriented[:, 2] = pts[:, 1]  # Y down
    return oriented


def _save_segmentation_overlays(
    rgb: np.ndarray,
    depth_m: np.ndarray,
    semantic_mask: np.ndarray,
    labels: list[str],
    output_path: Path,
    colors: np.ndarray,
) -> tuple[Path, Path]:
    """
    Persist RGB and depth overlays that visualize segmentation quality.

    Args:
        rgb (np.ndarray): RGB image used for inference.
        depth_m (np.ndarray): Depth image in meters.
        semantic_mask (np.ndarray): Semantic segmentation labels per pixel.
        labels (list[str]): Human-readable class labels for each object.
        output_path (Path): Base path used for writing visualization files.
        colors (np.ndarray): RGBA colors assigned per label.

    Returns:
        tuple[Path, Path]: Paths to the saved RGB and depth overlay images.
    """
    output_path = output_path.resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)

    rgb_overlay_path = output_path.with_name(f"{output_path.stem}_rgb_overlay{output_path.suffix}")
    depth_overlay_path = output_path.with_name(f"{output_path.stem}_depth_overlay{output_path.suffix}")

    overlay = np.zeros((*semantic_mask.shape, 4), dtype=np.float32)
    for idx in range(1, len(labels) + 1):
        mask = semantic_mask == idx
        if not np.any(mask):
            continue
        color = colors[(idx - 1) % len(colors)]
        overlay[mask, :3] = color[:3]
        overlay[mask, 3] = 0.45

    fig, ax = plt.subplots(figsize=(6, 5))
    ax.imshow(rgb)
    if labels:
        ax.imshow(overlay)
    ax.axis("off")
    fig.tight_layout(pad=0)
    fig.savefig(rgb_overlay_path, dpi=200)
    plt.close(fig)

    depth_display = np.array(depth_m, dtype=np.float32, copy=True)
    finite_mask = np.isfinite(depth_display)
    if np.any(finite_mask):
        vmin = float(np.nanmin(depth_display))
        vmax = float(np.nanpercentile(depth_display, 99))
        if not np.isfinite(vmin):
            vmin = 0.0
        if not np.isfinite(vmax) or np.isclose(vmax, vmin):
            vmax = vmin + 1.0
    else:
        vmin, vmax = 0.0, 1.0

    fig, ax = plt.subplots(figsize=(6, 5))
    im = ax.imshow(depth_display, cmap="viridis", vmin=vmin, vmax=vmax)
    fig.colorbar(im, ax=ax, fraction=0.046, pad=0.04, label="Depth (m)")
    if labels:
        ax.imshow(overlay)
    ax.axis("off")
    fig.tight_layout()
    fig.savefig(depth_overlay_path, dpi=200)
    plt.close(fig)

    return rgb_overlay_path, depth_overlay_path


def _visualize(
    point_clouds: np.ndarray,
    centroids: np.ndarray,
    labels: list[str],
    output_path: Path,
    colors: np.ndarray,
):
    """
    Render per-object point clouds with camera-aligned axes and save to disk.

    Args:
        point_clouds (np.ndarray): Sequence of per-object point clouds shaped (N_i, 3).
        centroids (np.ndarray): Array of centroid coordinates shaped (num_objects, 3).
        labels (list[str]): Labels corresponding to each point cloud.
        output_path (Path): Destination image path for the 3D scatter plot.
        colors (np.ndarray): RGBA colors assigned to each label for visualization.
    """
    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(111, projection="3d")
    rng = np.random.default_rng(0)
    seen_labels: set[str] = set()

    all_points_display: list[np.ndarray] = []
    for idx, (pc, centroid, label) in enumerate(zip(point_clouds, centroids, labels)):
        if pc is None or pc.size == 0:
            continue

        pc = np.asarray(pc, dtype=np.float32)
        # Sub-sample large point clouds for faster plotting.
        if pc.shape[0] > 5000:
            selection = rng.choice(pc.shape[0], size=5000, replace=False)
            pc_vis = pc[selection]
        else:
            pc_vis = pc

        pc_display = _reorient_for_camera_frame(pc_vis)
        centroid_display = _reorient_for_camera_frame(np.asarray(centroid, dtype=np.float32).reshape(1, 3))[0]

        color = colors[idx % len(colors)]
        legend_label = label if label not in seen_labels else None
        ax.scatter(
            pc_display[:, 0],
            pc_display[:, 1],
            pc_display[:, 2],
            s=3,
            color=color,
            alpha=0.6,
            label=legend_label,
        )
        ax.scatter(
            centroid_display[0],
            centroid_display[1],
            centroid_display[2],
            color=color,
            s=80,
            marker="*",
            edgecolors="k",
        )
        seen_labels.add(label)
        all_points_display.append(pc_display)

    if all_points_display:
        stacked = np.concatenate(all_points_display, axis=0)
        mins = stacked.min(axis=0)
        maxs = stacked.max(axis=0)
        ax.set_xlim(mins[0], maxs[0])
        ax.set_ylim(mins[1], maxs[1])
        ax.set_zlim(mins[2], maxs[2])

    ax.set_xlabel("X right (m)")
    ax.set_ylabel("Z forward (m)")
    ax.set_zlabel("Y down (m)")
    ax.invert_zaxis()
    ax.set_title("YOLOv11 Object Point Clouds")
    if seen_labels:
        ax.legend(loc="upper right")

    output_path = output_path.resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout()
    fig.savefig(output_path, dpi=200)
    plt.close(fig)


def run_example(config_path: Path, output_path: Path):
    """
    Execute the RGB-D perception pipeline given a configuration file.

    Args:
        config_path (Path): Path to the YAML file describing camera, depth, and perception settings.
        output_path (Path): Destination for the primary 3D point-cloud visualization.
    """
    cfg = _load_config(config_path)
    base_dir = config_path.parent

    camera_cfg = cfg["camera"]
    camera, transform_config = _build_camera(camera_cfg, base_dir)

    perception_cfg = cfg.get("perception", {})
    transform_kwargs = YoloPerception.load_constructor_kwargs(transform_config)

    yolo_kwargs: dict[str, object] = {
        "conf": float(perception_cfg.get("confidence", 0.25)),
        "iou": float(perception_cfg.get("iou", 0.45)),
        "classes": perception_cfg.get("classes"),
        "device": perception_cfg.get("device"),
        **transform_kwargs,
    }

    model_path_cfg = perception_cfg.get("model_path")
    if model_path_cfg is not None:
        model_path = Path(model_path_cfg)
        if not model_path.is_absolute():
            model_path = (base_dir / model_path).resolve()
        yolo_kwargs["model_path"] = model_path

    yolo = YoloPerception(camera, **yolo_kwargs)

    data_cfg = cfg["data"]
    rgb_path = (base_dir / data_cfg["rgb"]).resolve()
    depth_path = (base_dir / data_cfg["depth"]).resolve()

    rgb = _load_rgb(rgb_path)
    raw_depth = _load_depth(depth_path)
    depth_m = _convert_depth(raw_depth, cfg["depth_conversion"])

    semantic_mask, labels = yolo.detect_rgb(rgb)
    point_clouds, labels = yolo.get_object_point_clouds(depth_m, semantic_mask, labels)
    centroids = yolo.get_centroid(point_clouds)
    colors = _label_colors(len(labels))

    for label, centroid in zip(labels, centroids):
        if np.isnan(centroid).any():
            print(f"{label:>12s}: centroid unavailable (no depth overlap)")
        else:
            print(f"{label:>12s}: centroid {centroid}")

    _visualize(point_clouds, centroids, labels, output_path, colors)
    print(f"Saved point cloud visualization to {output_path}")

    rgb_overlay_path, depth_overlay_path = _save_segmentation_overlays(
        rgb, depth_m, semantic_mask, labels, output_path, colors
    )
    print(f"Saved RGB overlay to {rgb_overlay_path}")
    print(f"Saved depth overlay to {depth_overlay_path}")


def parse_args() -> argparse.Namespace:
    """
    Parse CLI arguments for the RGB-D YOLO perception example.

    Returns:
        argparse.Namespace: Parsed CLI arguments with `config` and `output` attributes.
    """
    parser = argparse.ArgumentParser(description="Run YOLOv11 segmentation on an RGB-D sample and export point clouds.")
    parser.add_argument(
        "--config",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "config" / "kitchen_example.yaml",
        help="YAML file describing camera parameters, depth conversion, and data paths.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("yolo_pointcloud.png"),
        help="Where to save the matplotlib visualization.",
    )
    return parser.parse_args()


def main():
    """
    Entry point for the command-line interface.

    Reads CLI arguments, runs the example, and writes visualization artifacts.
    """
    args = parse_args()
    run_example(args.config, args.output)


if __name__ == "__main__":
    main()
