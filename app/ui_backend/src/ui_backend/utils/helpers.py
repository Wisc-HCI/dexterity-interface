

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
        (list[dict]): List of objection dictionaries with the form:  {'name': ..., 'description': ..., 'pose': ..., grasp_pose: ..., dimensions: ....}
            - pose is centroid of object (with z at the bottom of object) in [x,y,z, qx, qy, qz, qw] in m
            - grasp_pose is [x,y,z, qx, qy, qz, qw] in m and is relative to centroid.
            - dimensions is [x (width), y (length), z (height)] in m
    """
    return [
        {"name": "cup", "description": "Small cup", "pose": [0.2, 0.1, 0.95, 0.0, 0.0, 0.0, 1.0],
         "grasp_pose": [0, 0.01, 0.05, 0, -0.818, 0.574, 0], 
         "dimensions": [0.05, 0.05, 0.08]},
        {"name": "bowl", "description": "Bowl.", "pose": [ 0.2, -0.2, 0.95, 0.0, 0.0, 0.0, 1.0],
         "grasp_pose": [0, 0.03, 0.05,  0, -0.818, 0.574, 0], 
         "dimensions": [0.136, 0.136, 0.0476]}
    ]