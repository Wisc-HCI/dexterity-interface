from sensor_interface.camera.rgbd_camera import RGBDCameraInterface
from planning.perception.perception import Perception

import numpy as np

# Ultralytics YOLOv11 (segmentation)
try:
    from ultralytics import YOLO
except Exception:
    YOLO = None

class YoloPerception(Perception):
    def __init__(
        self,
        camera_interface: RGBDCameraInterface,
        model_path: str = "yolo11n-seg.pt",
        conf: float = 0.25,
        iou: float = 0.45,
        classes: list[int] | None = None,
        device: str | None = None,  # e.g. "cuda" / "mps" / "cpu"
        transform_config_path: str | None = None,
    ):
        """
        Yolov11 perception
        """
        super().__init__(camera_interface, transform_config_path=transform_config_path)
        if YOLO is None:
            raise ImportError(
                "ultralytics not found. Please `pip install ultralytics` to use YOLOv11 segmentation."
            )

        self.model_path = model_path
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


    def detect_rgb(self, rgb: np.ndarray) -> tuple[np.ndarray, list[str]]:
        """
        Run segmentation/detection on an RGB image.

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
