from primitive_msgs_ros.action import Primitives

import json
import numpy as np


# TODO: Think about bimanual operations

CORE_PRIMITIVES = {'move_to_pose', 'grasp', 'release', 'home'}

def parse_prim_plan(prim_json:str) -> str:
    """
    Takes high-level (and core) primitive plan and parses it into purely core primitives using
    grasp, release, move_to_pose, home.

    Args:
        prim_json (str): Json string of list of core primitives.
    """
    prim_plan = json.loads(prim_json)
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
                prim['core_primitives'] = pick(params.get('arm'), params.get('initial_pose'), params.get('pour_orientation'), 
                                  params.get('pour_hold'))
            else:
                raise ValueError(f"Primitive '{name}' is not valid.")
        parsed_plan.append(prim)

    json_string = json.dumps(parsed_plan)
    return json_string


def pick(arm: str, grasp_pose: np.ndarray, end_position:np.ndarray=None) -> list[dict]:
    """
    Go to object, grasp, and translate. Keep same orientation after grasping.
    Args:
        arm (str): Which arm to use. Options: 'left', 'right'.
        grasp_pose (np.ndarray): (7,) Pose to grasp the object at in m/rad [x,y,z,qx,qy,qz,qw].
        end_position (np.ndarray): (3,) Position to move object to in m [x,y,z].
            If None, does not move object.
    Returns:
        (list[dict]): Array of core primitive dicts that make up prim.
    """
    prim = [
        {'name': 'move_to_pose',
         'parameters': {
             'arm': arm,
             'pose': grasp_pose
         }},
        {'name': 'grasp',
         'parameters': {
             'arm': arm,
             'pose': grasp_pose
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

def prim_plan_to_ros_msg(prim_plan:list[dict]) -> Primitives:
    """
    Take in the json string of primitives and turn into a ROS
    message that can be sent to primitive_action_handler_node.
    Args:
        prim_json (list[dict]): List of core primitives.
    Returns:
        (Primitives): ROS Message.

    """
    return Primitives()