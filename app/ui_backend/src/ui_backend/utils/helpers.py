import numpy as np

# def get_current_scene():
#     """
#     Returns the current scene description for planning and execution.
#     Returns:
#         (list[dict]): List of objection dictionaries with the form: 
#             {'name': ..., 'description': ..., 'position': ...}
#     """
#     return [
#         {"name": "cup", "description": "Small cup. Height: 0.08 (m). Use this grasp pose: [0.2, 0.11, 1, 0, -0.818, 0.574, 0]", "pose": [0.2, 0.1, 0.95, 0.0, 0.0, 0.0, 1.0]},
#         {"name": "bowl", "description": "Bowl. Height: 0.0476, Width:0.18 (m). Pour location: [0.2, -0.2, 1.15, 0, -0.788, -0.614, 0]", "pose": [ 0.2, -0.2, 0.95, 0.0, 0.0, 0.0, 1.0]},
#     ]


def get_current_scene():
    """
    Returns the current scene description for planning and execution.
    Returns:
        (list[dict]): List of object dictionaries with the form:  {'name': ..., 'description': ..., 'pose': ..., grasp_pose: ..., dimensions: ....}
            - pose (np.ndarray): (7,)  Centroid of object (with z at the bottom of object) in [x,y,z, qx, qy, qz, qw] in m
            - grasp_pose (np.ndarray): (7,) Pose to grasp relative to centroid [x,y,z, qx, qy, qz, qw] in m.
            - dimensions (np.ndarray): (3,) [x (width), y (length), z (height)] in m
    """

    # TODO: Instead treat grasp_pose as a couple points
    return [
        {"name": "cup", "description": "Small cup", "pose": np.array([0.2, 0.1, 0.95, 0.0, 0.0, 0.0, 1.0]),
         "grasp_pose": np.array([0, 0.01, 0.05, 0, -0.818, 0.574, 0]), 
         "dimensions": np.array([0.05, 0.05, 0.08])},
        
        {"name": "cup_1", "description": "Small cup", "pose": np.array([-0.3, 0.1, 0.95, 0.0, 0.0, 0.0, 1.0]),
         "grasp_pose": np.array([0, 0.01, 0.05, 0, -0.818, 0.574, 0]), 
         "dimensions": np.array([0.05, 0.05, 0.08])},
        {"name": "cup_2", "description": "Small cup", "pose": np.array([0.1, 0.0, 0.95, 0.0, 0.0, 0.0, 1.0]),
         "grasp_pose": np.array([0, 0.01, 0.05, 0, -0.818, 0.574, 0]), 
         "dimensions": np.array([0.05, 0.05, 0.08])},

        {"name": "bowl", "description": "Bowl.", "pose": np.array([ 0.2, -0.2, 0.95, 0.0, 0.0, 0.0, 1.0]),
         "grasp_pose": np.array([0, 0.03, 0.05,  0, -0.818, 0.574, 0]), 
         "dimensions": np.array([0.136, 0.136, 0.0476])},


        {"name": "bowl_1", "description": "Bowl.", "pose": np.array([ -0.3, -0.2, 0.95, 0.0, 0.0, 0.0, 1.0]),
         "grasp_pose": np.array([0, 0.03, 0.05,  0, -0.818, 0.574, 0]), 
         "dimensions": np.array([0.136, 0.136, 0.0476])}
    ]