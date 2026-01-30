from planning_py.examples.rgbd_yolo_example import _load_config, _build_camera, _load_rgb, _load_depth, _convert_depth, _visualize, _label_colors
from planning.perception.yolo_perception import YoloPerception
from sensor_interface.camera.rgbd_camera import CameraIntrinsics, RGBDCameraInterface

import argparse
from pathlib import Path
from typing import Tuple

import sys

import numpy as np


def run_example(config_path: Path, output_path: Path):
    """
    Execute the RGB-D pipeline given a configuration file. Saves pointclouds.

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
