import math
import random


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
        (list[dict]): List of objection dictionaries with the form: 
            {'name': ..., 'description': ..., 'position': ...}
    """

    CUP_POS = [0.2, 0.1, 0.95]
    CUP_RADIUS = 0.02
    CUP_HEIGHT = 0.05
    primary = [
        {"name": "cup", "description": "Small cup. Height: 0.08 (m). Use this grasp pose: [0.2, 0.11, 1, 0, -0.818, 0.574, 0]", 
            "pose": CUP_POS + [0.0, 0.0, 0.0, 1.0]},
        {"name": "bowl", "description": "Bowl. Height: 0.0476, Width:0.18 (m). Pour location: [0.2, -0.1, 1.1, 0, -0.788, -0.614, 0]", 
            "pose": [ 0.2, -0.2, 0.95, 0.0, 0.0, 0.0, 1.0]},
    ]

    filler  = []

    # for i in range(30):
    #     filler.append( 
    #         {"name": f"cube_{i}", 
    #          "description": "Sugar cube", 
    #          "pose": _sample_within_cylinder(CUP_POS,CUP_RADIUS, CUP_HEIGHT)}
    #     )

    if all_objects:
        return primary + filler
    else:
        return primary
    
