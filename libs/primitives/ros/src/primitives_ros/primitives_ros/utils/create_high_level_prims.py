from primitive_msgs_ros.msg import Primitive as PrimitiveMsg 

from geometry_msgs.msg import PoseStamped  

import numpy as np


# TODO: Think about bimanual operations

CORE_PRIMITIVES = {'move_to_pose', 'envelop_grasp', 'release', 'home'}

def parse_prim_plan(prim_plan:list[dict]) -> list[dict]:
    """
    Takes high-level (and core) primitive plan and parses it into purely core primitives using
    envelop_grasp, release, move_to_pose, home.

    Args:
        prim_plan (list[dict]): List of high level and core primitives in form of:
            [{'name': 'envelop_grasp', parameters: {'arm': 'left', pose: [0,0,0,0,0,0,1]} }, ...]

    Returns:
        (list[dict]): List of primitives with the high-level primitives containing
            all the core primitives in form of [{'name': 'envelop_grasp', parameters: {'arm': 'left', pose: [0,0,0,0,0,0,1]}, core_primitives: {...} }, ...]
    """

    parsed_plan = []
    
    for prim in prim_plan:
        name = prim.get('name')
        if name and name not in CORE_PRIMITIVES:
            params = prim.get('parameters')
            if not params:
                raise ValueError(f"Primitive '{name}' needs to have 'parameters' key")
            if name == "pick":
                prim['core_primitives'] = pick(params.get('arm'), params.get('grasp_pose'), params.get('end_position'))
            elif name == "pour":
                prim['core_primitives'] = pour(params.get('arm'), params.get('initial_pose'), params.get('pour_orientation'), 
                                  params.get('pour_hold'))
            else:
                raise ValueError(f"Primitive '{name}' is not valid.")
        parsed_plan.append(prim)


    return parsed_plan


def pick(arm: str, grasp_pose: np.ndarray, end_position:np.ndarray=None) -> list[dict]:
    """
    Go to object, envelop_grasp, and translate. Keep same orientation after grasping.
    Args:
        arm (str): Which arm to use. Options: 'left', 'right'.
        grasp_pose (np.ndarray): (7,) Pose to grasp the object at in m/rad [x,y,z,qx,qy,qz,qw].
        end_position (np.ndarray): (3,) Position to move object to in m [x,y,z].
            If None, does not move object.
    Returns:
        (list[dict]): Array of core primitive dicts that make up prim.
    """
    pre_grasp_pose = grasp_pose.copy()
    pre_grasp_pose[2] += 0.05 # 5 cm above
    prim = [
        {'name': 'move_to_pose',
         'parameters': {
             'arm': arm,
             'pose': pre_grasp_pose
         }},
        {'name': 'move_to_pose',
         'parameters': {
             'arm': arm,
             'pose': grasp_pose
         }},
        {'name': 'envelop_grasp',
         'parameters': {
             'arm': arm,
         }},
        {'name': 'move_to_pose',
         'parameters': {
             'arm': arm,
             'pose': end_position + grasp_pose[3:],
         }},
    ]

    return prim


def pour(arm: str, initial_pose: np.ndarray, pour_orientation:np.ndarray, pour_hold:float) -> list[dict]:
    """
    Angle robot to pour and then return to current position.
    Args:
        arm (str): Which arm to use. Options: 'left', 'right'.
        initial_pose (np.ndarray): (7,) Pose to start pour at m/rad [x,y,z,qx,qy,qz,qw].
        pour_orientation (np.ndarray): (5,) Orientation to pour at [qx,qy,qz,qw].
        pour_hold (float): Seconds to hold pour.
    Returns:
        (list[dict]): Array of core primitive dicts that make up prim.
    """
    # TODO: IMPLEMENT WAIT
    prim = [
        {'name': 'move_to_pose',
         'parameters': {
             'arm': arm,
             'pose': initial_pose
         }},
        {'name': 'move_to_pose',
         'parameters': {
             'arm': arm,
             'pose': initial_pose[:3] +  pour_orientation
         }},
        # {'name': 'wait',
        # 'parameters': {
        #     'arm': arm,
        #     'seconds': pour_hold
        # }},
        {'name': 'move_to_pose',
         'parameters': {
             'arm': arm,
             'pose': initial_pose
         }},
    ]

    return prim


def traverse_plan(current_prim_list: list[dict], current_idx_path: list[int],
                  flattened_prim_list: list[dict], flat_to_hierarchical_idx: dict[int, list[int]],
                  hierarchical_to_flat_idx: dict[tuple[int, ...], int]
                 ) -> tuple[list[dict], dict[int, list[int]], dict[tuple[int, ...], int]]:
    """
    Performs a depth-first traversal on a hierarchical primitive structure and flattens it
    into the list of core (leaf) primitives.
    Args:
        current_prim_list (list[dict]): List of primitives at the current level of the hierarchy.

        current_idx_path (list[int]): Hierarchical index path to the current level. Each element represents
            the index of a primitive at that depth in the hierarchy.

        flattened_prim_list (list[dict]):
            Accumulator list that is to be populated with flattened (core) primitives
            in execution order and returned.

        flat_to_hierarchical_idx (dict[int, list[int]]):
            Mapping from flattened primitive index to hierarchical index path to be returned.

        hierarchical_to_flat_idx (dict[int, list[int]]):
            Mapping from hierarchical primitive index to flattened index path to be returned.
    Returns:
        (list[dict]: Updated flattened primitive list.
        (dict[int, list[int]]): Updated mapping from flattened indices to hierarchical index paths.
        (dict[tuple(int), int]): Updated mapping from hierarchical index path to flattened indices.
    """
    
    for i, prim in enumerate(current_prim_list):
        idx_path = current_idx_path + [i]
        core_prims = prim.get('core_primitives')
        if core_prims:
            hierarchical_to_flat_idx[tuple(idx_path)] = len(flattened_prim_list)  # Maps parents to first child
            flattened_prim_list, flat_to_hierarchical_idx, hierarchical_to_flat_idx = traverse_plan(core_prims, idx_path, flattened_prim_list, flat_to_hierarchical_idx, hierarchical_to_flat_idx)
        else:    
            flattened_prim_list.append(prim)
            flat_idx = len(flattened_prim_list) - 1
            flat_to_hierarchical_idx[flat_idx] = idx_path
            hierarchical_to_flat_idx[tuple(idx_path)] = flat_idx
    
    return flattened_prim_list, flat_to_hierarchical_idx, hierarchical_to_flat_idx


def flatten_hierarchical_prims(prim_plan:list[dict]) -> tuple[list[dict], dict, dict]:
    """
    Flattens the high-level prim hierarchy into a list of only core primitives. 

    Args:
        prim_plan (list[dict]): List of primitives in the form of:
            [{'name': 'envelop_grasp', parameters: {'arm': 'left', pose: [0,0,0,0,0,0,1]}, core_primitives: {...} }, ...]

    Returns:
        (list[dict]): List of core primitives in the form of:
                    [{'name': 'envelop_grasp', parameters: {'arm': 'left', pose: [0,0,0,0,0,0,1]}]
        (dict): Dictionary where the flattened indexes are mapped to the hierarchical index. i.e. dict[3] = [1,3] where the value's index corresponds
            to the level in the hierarchy and the element at that index is the index in the hierarchy at that level.
        (dict): Dictionary where the hierarchical indexes (tuple) are mapped to the hierarchical index. i.e. dict[(1,3)] = 3 where the keys's index corresponds
            to the level in the hierarchy and the element at that index is the index in the hierarchy at that level.
    """

    return traverse_plan(prim_plan, [], [], {}, {})
    


def core_prim_to_ros_msg(core_prim:dict) -> PrimitiveMsg:
    """
    Creates ROS message out of core primitive dict.
    Args:
        core_prim (dict): Dict in form:
            {'name': 'envelop_grasp', parameters: {'arm': 'left', pose: [0,0,0,0,0,0,1]}
    Returns:
        (PrimitiveMsg): ROS primitive Message
    """

    prim_msg = PrimitiveMsg()

    prim_msg.type = core_prim['name']
    params = core_prim.get('parameters')
    if params is not None:
        if params.get("arm") is not None:
            prim_msg.arm = params['arm']

        pose = params.get("pose")
        if pose is not None:
            pose_msg = PoseStamped()
            pose_msg.pose.position.x = float(pose[0])
            pose_msg.pose.position.y = float(pose[1])
            pose_msg.pose.position.z = float(pose[2])
            pose_msg.pose.orientation.x = float(pose[3])
            pose_msg.pose.orientation.y = float(pose[4])
            pose_msg.pose.orientation.z = float(pose[5])
            pose_msg.pose.orientation.w = float(pose[6])

            prim_msg.pose = pose_msg

    return prim_msg


def prim_plan_to_ros_msg(flattened_prim_plan:list[dict]) -> list[PrimitiveMsg]:
    """
    Take in the flattened core primitives and turn into a ROS
    message list that can be sent to primitive_action_handler_node.
    Args:
        flattened_prim_plan (list[dict]): List of core primitives in the form of
            [{'name': 'envelop_grasp', parameters: {'arm': 'left', pose: [0,0,0,0,0,0,1]}]
    Returns:
        (list[PrimitiveMsg]): List of ROS Messages.
    """
    primitive_list = []
    for prim in flattened_prim_plan:
        primitive_list.append(core_prim_to_ros_msg(prim))
    return primitive_list
