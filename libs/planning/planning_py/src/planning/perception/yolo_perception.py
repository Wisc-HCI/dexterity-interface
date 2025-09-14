from sensor_interface.camera.rgbd_camera import RGBDCameraInterface
from planning.perception.perception import Perception

import numpy as np

class YoloPerception(Perception):
    def __init__(self, camera_interface: RGBDCameraInterface):
        """
        Yolov11 perception
        """
        super().__init__(camera_interface)


    def detect_rgb(self, rgb: np.ndarray, ) -> tuple[np.ndarray, list[str]]:
        """
        Run segmentation/detection on an RGB image.

        Returns:
            (np.ndarray): (H, W) Semantic mask with an object number 
                (corresponding to label index) for each pixel
            (list[str]): (num_objects) List of object label for each object
        """
        # TODO
        ...


    

    
if __name__ == "__main__":
    yolo = YoloPerception(None)