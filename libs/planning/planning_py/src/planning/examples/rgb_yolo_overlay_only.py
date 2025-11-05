"""
Run YOLO segmentation on RGB images and output RGB overlay pictures only.
Usage:
    python -m planning.examples.rgb_yolo_overlay_only \
        --config libs/planning/planning_py/src/planning/config/kitchen_images.yaml
"""

from __future__ import annotations
import argparse
from pathlib import Path
import sys
import yaml
import numpy as np
from PIL import Image
import matplotlib.pyplot as plt

_THIS_FILE = Path(__file__).resolve()
_PLANNING_SRC = _THIS_FILE.parents[2]
if str(_PLANNING_SRC) not in sys.path:
    sys.path.append(str(_PLANNING_SRC))

from planning.perception.yolo_perception import YoloPerception


def _load_rgb(path: Path) -> np.ndarray:
    """
    Load RGB image.
    
    Args:
        path (Path): Absolute or relative path to the image file.
    
    Returns:
        np.ndarray: (H, W, 3) uint8 array in RGB order.
    """

    return np.array(Image.open(path).convert("RGB"))


def _save_rgb_overlay(
    rgb: np.ndarray,
    semantic_mask: np.ndarray,
    labels: list[str],
    output_path: Path,
    add_legend: bool = False,
) -> None:
    """
    Save an RGB overlay visualization with FIXED, stable per-class colors and
    class labels placed directly on top of each mask.

    Args:
        rgb (np.ndarray): (H, W, 3) uint8 RGB image.
        semantic_mask (np.ndarray): (H, W) int mask where value (i+1) corresponds to labels[i].
        labels (list[str]): Class labels for indices in `semantic_mask` (1-indexed mapping).
        output_path (Path): Output path (file name stem is used; a `_rgb_overlay.png` is written).
        add_legend (bool): If True, add a legend showing class→color mapping.

    Returns:
        None
    """

    # Fixed colors for common kitchen objects
    CLASS_COLOR = {
        "person": (1.00, 0.10, 0.10),   # red
        "bowl":   (0.12, 0.47, 0.71),   # blue
        "cup":    (0.17, 0.63, 0.17),   # green
        "bottle": (0.58, 0.40, 0.74),   # purple
        "knife":  (0.55, 0.34, 0.29),   # brown
        "fork":   (0.90, 0.47, 0.76),   # pink
        "spoon":  (0.10, 0.62, 0.73),   # teal
        "plate":  (1.00, 0.50, 0.05),   # orange
        "chair":  (0.65, 0.65, 0.65),   # gray
        "table":  (0.80, 0.80, 0.36),   # olive
        "oven":   (0.74, 0.74, 0.13),
        "microwave": (0.40, 0.40, 0.88),
        "sink":   (0.30, 0.55, 0.80),
    }

    # Distinct, stable colors for common FOOD classes (normalized RGB 0-1)
    FOOD_COLOR = {
        "banana":     (0.96, 0.83, 0.16),  # yellow
        "apple":      (0.86, 0.12, 0.20),  # red
        "sandwich":   (0.72, 0.52, 0.32),  # tan
        "orange":     (0.98, 0.60, 0.10),  # orange-ish
        "broccoli":   (0.12, 0.50, 0.12),  # deep green
        "carrot":     (0.95, 0.45, 0.10),  # carrot orange
        "hot dog":    (0.80, 0.20, 0.10),  # dark red
        "pizza":      (0.90, 0.30, 0.15),  # sauce red
        "donut":      (0.75, 0.30, 0.60),  # magenta
        "cake":       (0.85, 0.70, 0.80),  # pastel pink
        "cookie":     (0.60, 0.45, 0.20),  # brown
        "bread":      (0.88, 0.72, 0.42),  # wheat
        "cucumber":   (0.20, 0.70, 0.35),  # green
        "egg":        (0.95, 0.95, 0.85),  # off-white
        "grape":      (0.45, 0.20, 0.55),  # purple
        "strawberry": (0.86, 0.12, 0.20),  # red
        "tomato":     (0.92, 0.20, 0.20),  # red
        "pineapple":  (0.95, 0.80, 0.00),  # yellow
        "lettuce":    (0.30, 0.75, 0.30),  # light green
        "onion":      (0.75, 0.55, 0.65),  # lilac
        "garlic":     (0.93, 0.93, 0.88),  # off-white
        "pepper":     (0.90, 0.20, 0.20),  # red pepper
    }

    # Fallback palette (stable by Python hash of label)
    TAB20 = plt.cm.tab20(np.linspace(0, 1, 20))

    def fallback_color(name: str) -> tuple[float, float, float]:
        idx = abs(hash(name)) % 20
        return tuple(TAB20[idx][:3])

    overlay = np.zeros((*semantic_mask.shape, 4), dtype=np.float32)
    alpha = 0.45
    annotations: list[tuple[int, int, str, tuple[float, float, float]]] = []

    for i, label in enumerate(labels):
        mask = semantic_mask == (i + 1)
        if not np.any(mask):
            continue

        lname = label.lower()
        color = FOOD_COLOR.get(lname, CLASS_COLOR.get(lname, fallback_color(lname)))

        overlay[mask, :3] = color
        overlay[mask, 3] = alpha

        ys, xs = np.where(mask)
        if xs.size > 0:
            cx, cy = int(xs.mean()), int(ys.mean())
            annotations.append((cx, cy, label, color))

    output_path.parent.mkdir(parents=True, exist_ok=True)
    out_file = output_path.with_name(f"{output_path.stem}_rgb_overlay.png")

    fig, ax = plt.subplots(figsize=(6, 5))
    ax.imshow(rgb)
    if labels:
        ax.imshow(overlay)
    ax.axis("off")

    if add_legend and labels:
        handles = []
        seen = set()
        for lbl in labels:
            lname = lbl.lower()
            if lname in seen:
                continue
            seen.add(lname)
            c = FOOD_COLOR.get(lname, CLASS_COLOR.get(lname, fallback_color(lname)))
            handles.append(
                plt.Line2D([0],[0], marker='s', linestyle='',
                           markerfacecolor=c, markeredgecolor='k',
                           markersize=8, label=lbl)
            )
        if handles:
            ax.legend(handles=handles, loc="upper right", framealpha=0.6)
    
    for (cx, cy, lbl, c) in annotations:
        ax.text(
            cx, cy, lbl,
            color="white", fontsize=9, weight="bold",
            va="center", ha="center",
            bbox=dict(facecolor=(0, 0, 0, 0.55), edgecolor="none", pad=1.8)
        )

    fig.tight_layout(pad=0)
    fig.savefig(out_file, dpi=200)
    plt.close(fig)
    print(f"Saved RGB overlay: {out_file}")


def run_rgb_only(config: Path) -> None:
    """
    Run YOLO segmentation on a directory of RGB images and write overlay PNGs.

    Args:
        config (Path): YAML config path. Expected keys:
            - data.images_dir (str): relative path to the input image directory.
            - output.dir (str, optional): relative path to the output directory (default: "rgb_out").
            - perception.* (dict): YOLO settings (confidence, iou, classes, device, model_path).

    Returns:
        None
    """

    cfg = yaml.safe_load(config.read_text()) or {}
    base = config.parent

    images_dir = (base / cfg["data"]["images_dir"]).resolve()
    output_dir = (base / cfg.get("output", {}).get("dir", "rgb_out")).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    yolo_cfg = cfg.get("perception", {})
    yolo_kwargs = {
        "conf": float(yolo_cfg.get("confidence", 0.25)),
        "iou": float(yolo_cfg.get("iou", 0.45)),
        "classes": yolo_cfg.get("classes"),
        "device": yolo_cfg.get("device"),
    }

    model_path_cfg = yolo_cfg.get("model_path")
    if model_path_cfg:
        mp = Path(model_path_cfg)
        if not mp.is_absolute():
            mp = (base / mp).resolve()
        yolo_kwargs["model_path"] = mp

    # YoloPerception requires a positional camera_interface; pass None for RGB-only use
    yolo = YoloPerception(None, **yolo_kwargs)

    exts = {".jpg", ".jpeg", ".png"}
    imgs = sorted([p for p in images_dir.iterdir() if p.suffix.lower() in exts])
    assert imgs, f"No images found in {images_dir}"

    for img in imgs:
        rgb = _load_rgb(img)
        semantic_mask, labels = yolo.detect_rgb(rgb)  # expected to return (mask, labels)
        _save_rgb_overlay(rgb, semantic_mask, labels, output_dir / img.name)


def parse_args() -> argparse.Namespace:
    """
    Parse command-line arguments for the RGB-only YOLO overlay script.

    Returns:
        argparse.Namespace: Parsed CLI arguments with field `config` (Path).
    """
    
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "--config",
        type=Path,
        required=True,
        help="YAML config specifying model path and images_dir",
    )
    return ap.parse_args()


if __name__ == "__main__":
    args = parse_args()
    run_rgb_only(args.config)
