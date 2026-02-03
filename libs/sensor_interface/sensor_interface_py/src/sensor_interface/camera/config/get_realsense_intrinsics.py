import pyrealsense2 as rs
import numpy as np
import json

WIDTH = 640
HEIGHT = 480
FPS = 30

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, WIDTH, HEIGHT, rs.format.rgb8, FPS)
config.enable_stream(rs.stream.depth, WIDTH, HEIGHT, rs.format.z16, FPS)

profile = pipeline.start(config)

# Get stream profiles
color_stream = profile.get_stream(rs.stream.color).as_video_stream_profile()
depth_stream = profile.get_stream(rs.stream.depth).as_video_stream_profile()

color_intr = color_stream.get_intrinsics()
depth_intr = depth_stream.get_intrinsics()

# Depth to color
extr = depth_stream.get_extrinsics_to(color_stream)

pipeline.stop()

# dict
output = {
    "color_intrinsics": {
        "width": color_intr.width,
        "height": color_intr.height,
        "fx": color_intr.fx,
        "fy": color_intr.fy,
        "cx": color_intr.ppx,
        "cy": color_intr.ppy,
        "distortion": list(color_intr.coeffs),
        "model": str(color_intr.model)
    },
    "depth_intrinsics": {
        "width": depth_intr.width,
        "height": depth_intr.height,
        "fx": depth_intr.fx,
        "fy": depth_intr.fy,
        "cx": depth_intr.ppx,
        "cy": depth_intr.ppy,
        "distortion": list(depth_intr.coeffs),
        "model": str(depth_intr.model)
    },
    "T_color_depth": np.array([
        extr.rotation[0:3],
        extr.rotation[3:6],
        extr.rotation[6:9]
    ]).tolist()
}

# Fix T-color-depth to 4x4 homogeneous
R = np.array(extr.rotation).reshape(3,3)
t = np.array(extr.translation)

T = np.eye(4)
T[:3,:3] = R
T[:3,3] = t

output["T_color_depth"] = T.tolist()

print(json.dumps(output, indent=2))
