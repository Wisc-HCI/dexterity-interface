from pyk4a import PyK4A, Config
from pprint import pprint

k4a = PyK4A(Config())  # default config just to open the device
k4a.start()
cal = k4a.calibration

def to_intrinsics(cam):
    p = cam.parameters.param
    return {
        "width": cam.resolution_width,
        "height": cam.resolution_height,
        "fx": p.fx, "fy": p.fy,
        "cx": p.cx, "cy": p.cy,
        "distortion": [p.k1, p.k2, p.p1, p.p2, p.k3],
    }

color = to_intrinsics(cal.color_camera_calibration)
depth = to_intrinsics(cal.depth_camera_calibration)
# 4x4 transform from depth optical -> color optical
T = cal.extrinsics[1].extrinsics  # depth to color; index 1 is depth, 0 is color in SDK ordering
import numpy as np
T_mat = np.eye(4)
R = T.rotation
T_mat[:3, :3] = [[R[0], R[1], R[2]],
                 [R[3], R[4], R[5]],
                 [R[6], R[7], R[8]]]
T_mat[:3, 3] = [T.translation.x / 1000.0,
                T.translation.y / 1000.0,
                T.translation.z / 1000.0]

pprint({"color_intrinsics": color, "depth_intrinsics": depth, "T_color_depth": T_mat.tolist()})
k4a.stop()
PY
