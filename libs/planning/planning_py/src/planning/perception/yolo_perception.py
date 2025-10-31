from sensor_interface.camera.rgbd_camera import RGBDCameraInterface
from planning.perception.perception import Perception

from pathlib import Path
import numpy as np
import yaml

# Ultralytics YOLOv11 (segmentation)
try:
    from ultralytics import YOLO
except Exception:
    YOLO = None

_DEFAULT_MODEL_PATH = Path(__file__).resolve().parent.parent / "yolo11n-seg.pt"


class YoloPerception(Perception):
    def __init__(
        self,
        camera_interface: RGBDCameraInterface,
        model_path: str | Path = _DEFAULT_MODEL_PATH,
        conf: float = 0.25,
        iou: float = 0.45,
        classes: list[int] | None = None,
        device: str | None = None,  # e.g. "cuda" / "mps" / "cpu"
        T_world_color: np.ndarray | None = None,
    ):
        """
        Initialize a YOLOv11-based perception pipeline for RGB-D sensing.

        Args:
            camera_interface (RGBDCameraInterface): Calibrated RGB-D camera interface.
            model_path (str | Path): Filesystem path to a YOLOv11 segmentation checkpoint.
            conf (float): Confidence threshold for detections (0-1).
            iou (float): IoU threshold for non-max suppression.
            classes (list[int] | None): Optional subset of class IDs to keep. `None` keeps all.
            device (str | None): Torch device identifier (e.g., "cuda", "mps", or "cpu").
            T_world_color (np.ndarray | None): (4, 4) transform mapping color-frame points into the
                world frame. Defaults to identity when omitted.
        """
        super().__init__(camera_interface, T_world_color=T_world_color)
        if YOLO is None:
            raise ImportError(
                "ultralytics not found. Please `pip install ultralytics` to use YOLOv11 segmentation."
            )

        resolved_model = Path(model_path)
        if not resolved_model.is_file():
            # Allow packaged checkpoints to be addressed relative to the module.
            candidate = (_DEFAULT_MODEL_PATH.parent / resolved_model).resolve()
            resolved_model = candidate if candidate.is_file() else resolved_model

        self.model_path = str(resolved_model)
        self.conf = conf
        self.iou = iou
        self.classes = classes
        self.device = device

        self.model = YOLO(self.model_path)
        if self.device is not None:
            # .to() is available on the underlying torch model
            try:
                self.model.to(self.device)
            except Exception:
                pass

    @classmethod
    def from_yaml(
        cls,
        camera_interface: RGBDCameraInterface,
        filename: str | Path,
        *,
        transform_config_path: str | Path | None = None,
    ) -> "YoloPerception":
        """
        Instantiate the perception pipeline directly from a YAML configuration file.

        The YAML file should contain the following keys:
            - model_path: (optional) path to the YOLO checkpoint.
            - confidence: (optional) detection confidence threshold.
            - iou: (optional) IoU threshold for non-max suppression.
            - classes: (optional) list of class IDs to retain.
            - device: (optional) Torch device string ("cuda", "cpu", "mps", ...).
            - transform_config: (optional) path to a YAML file containing `T_world_color`.

        Args:
            camera_interface (RGBDCameraInterface): Calibrated RGB-D camera interface.
            filename (str | Path): Path to the YAML file describing YOLO configuration.
            transform_config_path (str | Path | None): Optional override for the transform config
                file containing `T_world_color`. When omitted, the value from the YAML (if present)
                is used; otherwise the package default is applied.

        Returns:
            YoloPerception: Configured perception pipeline ready for inference.
        """
        path = Path(filename).expanduser()
        with path.open("r", encoding="utf-8") as stream:
            config = yaml.safe_load(stream) or {}

        base_dir = path.parent
        model_path = config.get("model_path", _DEFAULT_MODEL_PATH)
        model_path = Path(model_path)
        if not model_path.is_absolute():
            model_path = (base_dir / model_path).resolve()

        conf = float(config.get("confidence", 0.25))
        iou = float(config.get("iou", 0.45))
        classes = config.get("classes")
        device = config.get("device")

        if transform_config_path is None:
            transform_cfg = config.get("transform_config")
            if transform_cfg is not None:
                transform_config_path = (base_dir / transform_cfg).resolve()

        constructor_kwargs = cls.load_constructor_kwargs(transform_config_path)

        return cls(
            camera_interface,
            model_path=model_path,
            conf=conf,
            iou=iou,
            classes=classes,
            device=device,
            **constructor_kwargs,
        )


    def detect_rgb(self, rgb: np.ndarray) -> tuple[np.ndarray, list[str]]:
        """
        Run segmentation/detection on an RGB image.

        Args:
            rgb (np.ndarray): (H, W, 3) RGB image expressed in uint8 or float format.

        Returns:
            (np.ndarray): (H, W) Semantic mask with an object number 
                (corresponding to label index) for each pixel
            (list[str]): (num_objects) List of object label for each object
        """
        if rgb is None or not isinstance(rgb, np.ndarray) or rgb.ndim != 3 or rgb.shape[2] != 3:
            raise ValueError("`rgb` must be an (H, W, 3) RGB numpy array")

        H, W = rgb.shape[:2]
        # YOLOv11 segmentation inference
        results = self.model.predict(
            rgb,
            task="segment",
            verbose=False,
            conf=self.conf,
            iou=self.iou,
            classes=self.classes,
        )
        res = results[0]

        # Initialize outputs
        semantic = np.zeros((H, W), dtype=np.int32) # 0 = background
        labels: list[str] = []

        # No masks -> return empty result
        masks = getattr(res, "masks", None)
        if masks is None or getattr(masks, "data", None) is None or masks.data.shape[0] == 0:
            return semantic, labels

        # Gather boxes/masks/classes/scores
        masks_tensor = masks.data   # (N, Hm, Wm) torch.Tensor of 0/1 or probs
        num_masks = masks_tensor.shape[0]

        boxes = getattr(res, "boxes", None)
        if boxes is not None and getattr(boxes, "cls", None) is not None:
            cls_ids = boxes.cls.detach().cpu().numpy().astype(int)
            cls_ids = cls_ids[:num_masks]
        else:
            cls_ids = np.zeros((num_masks,), dtype=int)

        if boxes is not None and getattr(boxes, "conf", None) is not None:
            scores = boxes.conf.detach().cpu().numpy()
            scores = scores[:num_masks]
        else:
            scores = np.ones((num_masks,), dtype=float)

        # Order by descending confidence so that higher-confidence objects overwrite lower ones on overlaps
        order = np.argsort(-scores)
        masks_np = masks_tensor.cpu().numpy().astype(np.uint8)  # (N, Hm, Wm)

        # Ultralytics usually returns masks already resized to input size; guard just in case
        need_resize = masks_np.shape[1:] != (H, W)
        if need_resize:
            import cv2

        obj_idx = 1
        for i in order:
            m = masks_np[i]
            if need_resize:
                m = cv2.resize(m, (W, H), interpolation=cv2.INTER_NEAREST)

            # binarize
            m = (m > 0).astype(np.uint8)
            if m.sum() == 0:
                continue

            # Assign object index wherever mask is 1 (overwrites lower-confidence overlaps)
            semantic[m == 1] = obj_idx

            # Map class id -> class name
            names = getattr(res, "names", None)
            if names is None:
                names = getattr(self.model, "names", None)

            label = None
            if isinstance(names, dict):
                label = names.get(int(cls_ids[i]))
            elif isinstance(names, (list, tuple)):
                idx = int(cls_ids[i])
                if 0 <= idx < len(names):
                    label = names[idx]

            labels.append(label if label is not None else str(int(cls_ids[i])))
            obj_idx += 1

        return semantic, labels
    
    
if __name__ == "__main__":
    yolo = YoloPerception(None)
