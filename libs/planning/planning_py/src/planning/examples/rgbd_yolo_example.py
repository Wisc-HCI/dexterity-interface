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
        super().__init__(color_intrinsics, depth_intrinsics, T_color_depth)

    def start(self, *args, **kwargs):  # pragma: no cover - example only
        raise NotImplementedError("Static camera does not stream frames")

    def stop(self):  # pragma: no cover - example only
        raise NotImplementedError("Static camera does not stream frames")

    def is_running(self) -> bool:  # pragma: no cover - example only
        return False

    def latest(self):  # pragma: no cover - example only
        raise NotImplementedError("Static camera does not stream frames")


def _load_config(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as fh:
        return yaml.safe_load(fh)


def _build_camera(camera_cfg: dict, base_dir: Path) -> Tuple[StaticRGBDCamera, Path | None]:
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
    return np.array(Image.open(path).convert("RGB"))


def _load_depth(path: Path) -> np.ndarray:
    # Preserve full precision for raw disparity values
    return np.array(Image.open(path), dtype=np.float32)


def _convert_depth(raw_depth: np.ndarray, cfg: dict) -> np.ndarray:
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


def _visualize(point_clouds: np.ndarray, centroids: np.ndarray, labels: list[str], output_path: Path):
    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(111, projection="3d")
    rng = np.random.default_rng(0)

    colors = plt.cm.tab10(np.linspace(0, 1, max(len(point_clouds), 1)))
    seen_labels: set[str] = set()

    all_points = []
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

        legend_label = label if label not in seen_labels else None
        ax.scatter(pc_vis[:, 0], pc_vis[:, 1], pc_vis[:, 2], s=3, color=colors[idx], alpha=0.6, label=legend_label)
        ax.scatter(centroid[0], centroid[1], centroid[2], color=colors[idx], s=80, marker="*", edgecolors="k")
        seen_labels.add(label)
        all_points.append(pc)

    if all_points:
        stacked = np.concatenate(all_points, axis=0)
        mins = stacked.min(axis=0)
        maxs = stacked.max(axis=0)
        ax.set_xlim(mins[0], maxs[0])
        ax.set_ylim(mins[1], maxs[1])
        ax.set_zlim(mins[2], maxs[2])

    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    ax.set_title("YOLOv11 Object Point Clouds")
    if seen_labels:
        ax.legend(loc="upper right")

    output_path = output_path.resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout()
    fig.savefig(output_path, dpi=200)
    plt.close(fig)


def run_example(config_path: Path, output_path: Path):
    cfg = _load_config(config_path)
    base_dir = config_path.parent

    camera_cfg = cfg["camera"]
    camera, transform_config = _build_camera(camera_cfg, base_dir)

    perception_cfg = cfg.get("perception", {})
    yolo = YoloPerception(
        camera,
        model_path=perception_cfg.get("model_path", "yolo11n-seg.pt"),
        conf=float(perception_cfg.get("confidence", 0.25)),
        iou=float(perception_cfg.get("iou", 0.45)),
        classes=perception_cfg.get("classes"),
        device=perception_cfg.get("device"),
        transform_config_path=transform_config,
    )

    data_cfg = cfg["data"]
    rgb_path = (base_dir / data_cfg["rgb"]).resolve()
    depth_path = (base_dir / data_cfg["depth"]).resolve()

    rgb = _load_rgb(rgb_path)
    raw_depth = _load_depth(depth_path)
    depth_m = _convert_depth(raw_depth, cfg["depth_conversion"])

    semantic_mask, labels = yolo.detect_rgb(rgb)
    point_clouds, labels = yolo.get_object_point_clouds(depth_m, semantic_mask, labels)
    centroids = yolo.get_centroid(point_clouds)

    for label, centroid in zip(labels, centroids):
        if np.isnan(centroid).any():
            print(f"{label:>12s}: centroid unavailable (no depth overlap)")
        else:
            print(f"{label:>12s}: centroid {centroid}")

    _visualize(point_clouds, centroids, labels, output_path)
    print(f"Saved point cloud visualization to {output_path}")


def parse_args() -> argparse.Namespace:
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
    args = parse_args()
    run_example(args.config, args.output)


if __name__ == "__main__":
    main()
