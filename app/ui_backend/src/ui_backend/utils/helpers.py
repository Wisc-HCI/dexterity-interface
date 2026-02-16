import numpy as np
import math
import random

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



def _sample_within_cylinder(position:list, radius:float, height: float):
    """
    Generate a random location within a cylinder at positioned at the
    given position with the given radius and height.
    Args:
        position(list): (3,) position of cylinder [x,y,z] in m where x,y are at center
        of cylinder and z is at its base.
        radius (float): Radius of the cylinder in m
        height (float): Height of cylinder in m
    """
    r = radius * math.sqrt(random.random())
    theta = random.uniform(0, 2 * math.pi)

    x = position[0] + r * math.cos(theta)
    y = position[1] + r * math.sin(theta)
    z = position[2] + random.uniform(0.0, height)

    return [x, y, z, 0.0, 0.0, 0.0, 1.0]


def get_current_scene(all_objects:bool=False):
    """
    Returns the current scene description for planning and execution.
    Args:
        all_objects (bool): True if return all objects in the scene. False if only 
        return the major objects (used for LLM), not the minor/filling ones.
    Returns:
        (list[dict]): List of object dictionaries with the form:  {'name': ..., 'description': ..., 'pose': ..., grasp_pose: ..., dimensions: ....}
            - pose (np.ndarray): (7,)  Centroid of object (with z at the bottom of object) in [x,y,z, qx, qy, qz, qw] in m
            - grasp_pose (np.ndarray): (7,) Pose to grasp relative to centroid [x,y,z, qx, qy, qz, qw] in m.
            - dimensions (np.ndarray): (3,) [x (width), y (length), z (height)] in m
    """

    # TODO: Instead treat grasp_pose as a couple points
    return [
        # {
        # "name": "barrier", "description": "Barrier", "pose": np.array([0.0, 0.0, 0.95, 0.0, 0.0, 0.0, 1.0]),
        #  "grasp_pose": np.array([0, 0, 0, 0, 0, 0, 1]), # None 
        #  "dimensions": np.array([0.1, 0.5, 0.5])
        # },
        {"name": "cup", "description": "Small cup", "pose": np.array([0.2, 0.1, 0.9369, 0.0, 0.0, 0.0, 1.0]),
         "grasp_pose": np.array([0, 0.01, 0.04, 0, -0.818, 0.574, 0]), 
         "dimensions": np.array([0.05, 0.05, 0.08])},
        
        {"name": "cup_1", "description": "Small cup", "pose": np.array([-0.3, 0.1, 0.9369, 0.0, 0.0, 0.0, 1.0]),
         "grasp_pose": np.array([0, 0.01, 0.04, 0, -0.818, 0.574, 0]), 
         "dimensions": np.array([0.05, 0.05, 0.08])},
        {"name": "cup_2", "description": "Small cup", "pose": np.array([0.1, 0.0, 0.9369, 0.0, 0.0, 0.0, 1.0]),
         "grasp_pose": np.array([0, 0.01, 0.04, 0, -0.818, 0.574, 0]), 
         "dimensions": np.array([0.05, 0.05, 0.08])},

        {"name": "bowl", "description": "Bowl.", "pose": np.array([ 0.2, -0.2, 0.9369, 0.0, 0.0, 0.0, 1.0]),
         "grasp_pose": np.array([0, 0.03, 0.03,  0, -0.818, 0.574, 0]), 
         "dimensions": np.array([0.136, 0.136, 0.0476])},


        {"name": "bowl_1", "description": "Bowl.", "pose": np.array([ -0.3, -0.2, 0.9369, 0.0, 0.0, 0.0, 1.0]),
         "grasp_pose": np.array([0, 0.03, 0.03,  0, -0.818, 0.574, 0]), 
         "dimensions": np.array([0.136, 0.136, 0.0476])}
    ]


    # CUP_POS = [0.2, 0.1, 0.95]
    # CUP_RADIUS = 0.02
    # CUP_HEIGHT = 0.05
    # primary = [
    #     {"name": "cup", "description": "Small cup. Height: 0.08 (m). Use this grasp pose: [0.2, 0.11, 1, 0, -0.818, 0.574, 0]", 
    #         "pose": CUP_POS + [0.0, 0.0, 0.0, 1.0]},
    #     {"name": "bowl", "description": "Bowl. Height: 0.0476, Width:0.18 (m). Pour location: [0.2, -0.1, 1.1, 0, -0.788, -0.614, 0]", 
    #         "pose": [ 0.2, -0.2, 0.95, 0.0, 0.0, 0.0, 1.0]},
    # ]

    # filler  = []

    # # for i in range(30):
    # #     filler.append( 
    # #         {"name": f"cube_{i}", 
    # #          "description": "Sugar cube", 
    # #          "pose": _sample_within_cylinder(CUP_POS,CUP_RADIUS, CUP_HEIGHT)}
    # #     )

    # if all_objects:
    #     return primary + filler
    # else:
    #     return primary

