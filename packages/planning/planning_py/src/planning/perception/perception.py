from sensor_interface.camera.rgbd_camera import RGBDCameraInterface

from abc import abstractmethod
import numpy as np

class Perception:
    def __init__(self, camera_interface: RGBDCameraInterface):
        """
        Object detection and localization.
        """
        self.camera_interface = camera_interface


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

    def get_object_point_clouds(self, depth:np.ndarray, semantic_mask:np.ndarray, 
                                labels:list[str]) -> tuple[np.ndarray, list[str]]:
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
        # TODO: Use camera  properties to do this
        ...


    def get_centroid(self, point_clouds:np.ndarray) -> np.ndarray:
        """
        Get the single center of the point cloud.
        Args:
            point_clouds (np.ndarray): (num_objects, N, 3) List of object point clouds
        Returns:
            (np.ndarray): (num_objects, 3) Array of center point of each point cloud [[x, y, z]]
        """ 
        # TODO: Use geometric mean or something fancy like ransac.
        ...

    
    

    
