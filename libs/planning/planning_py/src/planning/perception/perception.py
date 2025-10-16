from sensor_interface.camera.rgbd_camera import RGBDCameraInterface

from abc import abstractmethod
from pathlib import Path
import numpy as np
import yaml


_DEFAULT_TRANSFORM_CONFIG = (
    Path(__file__).resolve().parent.parent / "config" / "camera_transforms.yaml"
)


class Perception:
    def __init__(self, camera_interface: RGBDCameraInterface, transform_config_path: str | None = None):
        """
        Object detection and localization.
        """
        self.camera_interface = camera_interface
        self._transform_config_path = (
            Path(transform_config_path).expanduser()
            if transform_config_path is not None
            else _DEFAULT_TRANSFORM_CONFIG
        )
        self.T_world_color = self._load_world_transform()


    @abstractmethod
    def detect_rgb(self, rgb: np.ndarray, ) -> tuple[np.ndarray, list[str]]:
        """
        Run segmentation/detection on an RGB image.

        Returns:
            (np.ndarray): (H, W) Semantic mask with an object number 
                (corresponding to label index) for each pixel
            (list[str]): (num_objects) List of object label for each object
        """
        ...

    def get_object_point_clouds(
        self,
        depth: np.ndarray,
        semantic_mask: np.ndarray,
        labels: list[str],
    ) -> tuple[np.ndarray, list[str]]:
        """
        Extract per-object point clouds from a depth map and corresponding
        semantic segmentation mask.

        depth (np.ndarray): (H, W) depth map in meters.
        semantic_mask (np.ndarray): (H, W) Semantic mask with an object number 
                (corresponding to label index) for each pixel.
        labels (list[str]): List of object label for each object.

        Returns:
            (np.ndarray): (num_objects, N, 3) Array of object point clouds
            (list[str]):(num_objects) List of object label for each point cloud
        """
        if self.camera_interface is None:
            raise RuntimeError("Camera interface is required for point cloud extraction")

        if depth is None or semantic_mask is None:
            raise ValueError("`depth` and `semantic_mask` must both be provided")

        if depth.ndim != 2:
            raise ValueError("`depth` must be a 2-D array")
        if semantic_mask.ndim != 2:
            raise ValueError("`semantic_mask` must be a 2-D array")

        depth = depth.astype(np.float32, copy=False)
        semantic_mask = semantic_mask.astype(np.int32, copy=False)

        depth_intr = self.camera_interface.depth_intrinsics
        color_intr = self.camera_interface.color_intrinsics
        T_color_depth = self.camera_interface.T_color_depth

        if depth_intr is None or color_intr is None or T_color_depth is None:
            raise RuntimeError("Camera intrinsics/extrinsics must be initialized")

        H_d, W_d = depth.shape
        H_c, W_c = semantic_mask.shape

        if (depth_intr.width, depth_intr.height) != (W_d, H_d):
            raise ValueError("Depth intrinsics do not match the depth image dimensions")
        if (color_intr.width, color_intr.height) != (W_c, H_c):
            raise ValueError("Color intrinsics do not match the semantic mask dimensions")

        v_coords, u_coords = np.indices((H_d, W_d), dtype=np.float32)
        z = depth.reshape(-1)
        valid_depth = np.isfinite(z) & (z > 0)

        if not np.any(valid_depth):
            empty = np.empty((0, 3), dtype=np.float32)
            return np.array([empty for _ in labels], dtype=object), labels

        u_flat = u_coords.reshape(-1)
        v_flat = v_coords.reshape(-1)

        x = (u_flat - depth_intr.cx) * z / depth_intr.fx
        y = (v_flat - depth_intr.cy) * z / depth_intr.fy

        ones = np.ones_like(z)
        pts_depth = np.stack((x, y, z, ones), axis=0)

        valid_indices = np.nonzero(valid_depth)[0]

        pts_color = T_color_depth @ pts_depth[:, valid_depth]
        Z_c = pts_color[2]
        positive_mask = Z_c > 0
        if not np.any(positive_mask):
            empty = np.empty((0, 3), dtype=np.float32)
            return np.array([empty for _ in labels], dtype=object), labels

        valid_indices = valid_indices[positive_mask]
        pts_color = pts_color[:, positive_mask]

        u_proj = (pts_color[0] * color_intr.fx) / pts_color[2] + color_intr.cx
        v_proj = (pts_color[1] * color_intr.fy) / pts_color[2] + color_intr.cy

        u_int = np.rint(u_proj).astype(int)
        v_int = np.rint(v_proj).astype(int)

        inside_mask = (
            (u_int >= 0)
            & (u_int < W_c)
            & (v_int >= 0)
            & (v_int < H_c)
        )

        if not np.any(inside_mask):
            empty = np.empty((0, 3), dtype=np.float32)
            return np.array([empty for _ in labels], dtype=object), labels

        u_int = u_int[inside_mask]
        v_int = v_int[inside_mask]
        valid_indices = valid_indices[inside_mask]
        pts_color = pts_color[:, inside_mask]

        depth_mask_flat = np.zeros(H_d * W_d, dtype=np.int32)
        object_ids = semantic_mask[v_int, u_int]
        depth_mask_flat[valid_indices] = object_ids
        _depth_mask = depth_mask_flat.reshape(H_d, W_d)  # currently unused but handy for debugging

        pts_world = self.T_world_color @ pts_color
        pts_world_cart = pts_world[:3].T.astype(np.float32, copy=False)

        point_clouds: list[np.ndarray] = []
        num_known = len(labels)
        for obj_idx in range(1, num_known + 1):
            selection = object_ids == obj_idx
            if not np.any(selection):
                point_clouds.append(np.empty((0, 3), dtype=np.float32))
            else:
                point_clouds.append(pts_world_cart[selection])

        return np.array(point_clouds, dtype=object), labels


    def get_centroid(self, point_clouds:np.ndarray) -> np.ndarray:
        """
        Get the single center of the point cloud.
        Args:
            point_clouds (np.ndarray): (num_objects, N, 3) List of object point clouds
        Returns:
            (np.ndarray): (num_objects, 3) Array of center point of each point cloud [[x, y, z]]
        """ 
        if point_clouds is None:
            raise ValueError("`point_clouds` must be provided")

        centroids: list[np.ndarray] = []
        for pc in point_clouds:
            if pc is None or pc.size == 0:
                centroids.append(np.full(3, np.nan, dtype=np.float32))
                continue

            pc_array = np.asarray(pc, dtype=np.float32)
            if pc_array.ndim != 2 or pc_array.shape[1] != 3:
                raise ValueError("Each point cloud must have shape (N, 3)")

            centroids.append(pc_array.mean(axis=0))

        return np.stack(centroids, axis=0)

    def _load_world_transform(self) -> np.ndarray:
        """Load the color camera to world transform from YAML, defaulting to identity."""
        config_path = self._transform_config_path
        if not config_path.exists():
            return np.eye(4, dtype=np.float32)

        with open(config_path, "r", encoding="utf-8") as stream:
            data = yaml.safe_load(stream) or {}

        matrix = data.get("T_world_color")
        if matrix is None:
            return np.eye(4, dtype=np.float32)

        arr = np.array(matrix, dtype=np.float32)
        if arr.shape != (4, 4):
            raise ValueError("`T_world_color` must be a 4x4 homogeneous transform")

        return arr

    
    

    
