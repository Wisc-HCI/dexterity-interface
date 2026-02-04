from primitives_ros.utils.transformation_utils import pose_to_transformation, transformation_to_pose
from primitive_msgs_ros.msg import Primitive as PrimitiveMsg 

import copy

from scipy.spatial.transform import Rotation as R, Slerp

import numpy as np

from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped  




# TODO: Think about bimanual operations

CORE_PRIMITIVES = {'move_to_pose', 'envelop_grasp', 'release', 'home'}

def parse_prim_plan(prim_plan:list[dict], objects:list[str] = []) -> list[dict]:
    """
    Takes high-level (and core) primitive plan and parses it into purely core primitives using
    envelop_grasp, release, move_to_pose, home.

    Args:
        prim_plan (list[dict]): List of high level and core primitives in form of:
            [{'name': 'envelop_grasp', parameters: {'arm': 'left', pose: [0,0,0,0,0,0,1]} }, ...]

        (list[dict]): List of object dictionaries with the form:  {'name': ..., 'description': ..., 'pose': ..., grasp_pose: ..., dimensions: ....}
            - pose (np.ndarray): (7,)  Centroid of object (with z at the bottom of object) in [x,y,z, qx, qy, qz, qw] in m
            - grasp_pose (np.ndarray): (7,) Pose to grasp relative to centroid [x,y,z, qx, qy, qz, qw] in m.
            - dimensions (np.ndarray): (3,) [x (width), y (length), z (height)] in m

    Returns:
        (list[dict]): List of primitives with the high-level primitives containing
            all the core primitives in form of [{'name': 'envelop_grasp', parameters: {'arm': 'left', pose: [0,0,0,0,0,0,1]}, core_primitives: {...} }, ...]
    """

    parsed_plan = []

    tracked_objects = object_list_to_dict(objects)


    
    for prim in prim_plan:
        name = prim.get('name')
        if name and name  in CORE_PRIMITIVES:
            prim['core_primitives'] = None
            tracked_objects = update_object_tracking(prim, tracked_objects)
        elif name:
            params = prim.get('parameters')
            if not params:
                raise ValueError(f"Primitive '{name}' needs to have 'parameters' key")
            if name == "pick":
                prim = pick(prim, tracked_objects)
            elif name == "pour":
                prim = pour(prim, tracked_objects)
            else:
                raise ValueError(f"Primitive '{name}' is not valid.")
            
            # TODO: Determine if this is best way to do this
            for core_prim in prim["core_primitives"]:
                print("PRIM", prim["name"])
                tracked_objects = update_object_tracking(core_prim, tracked_objects)

        parsed_plan.append(prim)


    return parsed_plan


def update_object_tracking(core_prim:dict, tracked_objects:dict) -> dict:
    """
    Update tracked object state based on the execution of a core primitive.
    Only primitives that reference an object ('object' parameter) affect
    tracking state.

    Args:
        core_prim (dict): A core primitive dict with keys 'name', 'parameters'

        tracked_objects (dict):
            Dictionary mapping object names to tracked object state.

    Returns:
        (dict): Updated tracked_objects dictionary.
    """
    
    prim_name = core_prim["name"]
    params = core_prim["parameters"]
    prim_arm = params.get("arm")
    obj_name = params.get("object")
    
    if obj_name is None:
        return tracked_objects

    obj = tracked_objects[obj_name]
    obj_arm = obj["grasped_by"]

    
    if prim_name == "home":
        obj["grasped_by"] = None
    elif prim_name == "envelop_grasp":
        obj["grasped_by"] = prim_arm
    elif prim_arm == obj_arm:
        if prim_name == "release":
            obj["grasped_by"] = None
        elif prim_name == "move_to_pose":

            T_world_ee = pose_to_transformation(params["pose"])
            new_T_centroid_grasp = T_world_ee @ np.linalg.inv( obj["T_centroid_grasp"])

            collided, collided_obj, _ = check_collisions_along_interpolation(obj, new_T_centroid_grasp, tracked_objects)
            if collided:
                collided_obj_name = collided_obj["name"]
                print(f"COLLIDED with {collided_obj_name} during {prim_name}")

            obj["T_world_centroid"] = new_T_centroid_grasp

    tracked_objects[obj_name] = obj

    return tracked_objects


def object_list_to_dict(objects_list:list[dict]) -> dict:
    """
    Convert a list of object descriptions into an internal tracked-object
    dictionary used for planning and collision checking.
    Args:
        objects_list (list[dict]): List of object dictionaries with the form:  {'name': ..., 'description': ..., 'pose': ..., grasp_pose: ..., dimensions: ....}
            - pose (np.ndarray): (7,)  Centroid of object (with z at the bottom of object) in [x,y,z, qx, qy, qz, qw] in m
            - grasp_pose (np.ndarray): (7,) Pose to grasp relative to centroid [x,y,z, qx, qy, qz, qw] in m.
            - dimensions (np.ndarray): (3,) [x (width), y (length), z (height)] in m
    Returns:
        (dict): Objects in form of {'OBJECT_NAME'': {'grasped_by':'', 'T_world_centroid':[], 'T_centroid_grasp'}}
            - grasped_by (str): 'left' or 'right based on what is gripping object

            TODO: swap to object??
    """
          

    tracked_objects = {}

    for obj in objects_list:
        name = obj["name"]
        
        formatted_obj = {
            "name": name,
            "grasped_by": None,  # Left or right
            "T_world_centroid": pose_to_transformation(obj["pose"]),
            "T_centroid_grasp": pose_to_transformation(obj["grasp_pose"]), 
            # Bounding box of object with 8 corners, relative to centroid with 3cm buffer
            "T_centroid_corners": dimensions_to_bounding_box_transforms(obj["dimensions"], 0.03)
           
        }

        tracked_objects[name] = formatted_obj

    return tracked_objects


def calculate_minimum_clearance_height(moving_obj: dict, stationary_obj: dict, 
                                       clearance_buffer: float = 0.03) -> float:
    """
    Calculate the minimum distance adjustment needed to clear one object over another (dz).
    
    Args:
        moving_obj (dict): Object being moved (with T_world_centroid)
        stationary_obj (dict): Object to clear over
        clearance_buffer: Additional safety margin in meters (default 3cm)
    
    Returns:
        (float) Minimum distance (dz) in m.
    """

    # Top of stationary object in world frame
    stat_T_world_corners = stationary_obj["T_world_centroid"] @ stationary_obj["T_centroid_corners"]
    stat_top_z = np.max(stat_T_world_corners[:, 2, 3])
    
    # Bottom of object in world frame
    moving_T_world_corners = moving_obj["T_world_centroid"] @ moving_obj["T_centroid_corners"]
    moving_bottom_z = np.min(moving_T_world_corners[:, 2, 3])
    
    dz = stat_top_z - moving_bottom_z + clearance_buffer

    return max(0, dz)


def dimensions_to_bounding_box_transforms(dimensions:np.ndarray, buffer:float=0):
    """
    Generate 8 homogeneous SE(3) transforms corresponding to the
    corners of an object's bounding box, expressed in the centroid frame (T_centroid_corners).

    The box follows IsaacSim convention:
      - Object frame origin is centered in X/Y
      - Bottom face lies on Z = 0
      - Z points upward
      - dimensions = [dx, dy, dz] in meters

    Args:
        dimensions (np.ndarray): (3,) [width (x), length (y), height (z)] in meters.
        buffer (float): buffer to add to each dimension (m). Good for collision detection

    Returns:
        (np.ndarray): (8, 4, 4) Array of 8 homogeneous 4x4 transformation matrices
    """

    dx, dy, dz = dimensions + buffer
    hx, hy, hz = dx / 2, dy / 2, dz

    translations = np.array([
        [ hx,  hy,  hz],
        [ hx,  hy, 0],
        [ hx, -hy,  hz],
        [ hx, -hy, 0],
        [-hx,  hy,  hz],
        [-hx,  hy, 0],
        [-hx, -hy,  hz],
        [-hx, -hy, 0],
    ])


    # # Actually centered (for future use if needed)
    # hx, hy, hz = dx / 2, dy / 2, d/2
    # translations = np.array([
    #     [ hx,  hy,  hz],
    #     [ hx,  hy, -hz],
    #     [ hx, -hy,  hz],
    #     [ hx, -hy, -hz],
    #     [-hx,  hy,  hz],
    #     [-hx,  hy, -hz],
    #     [-hx, -hy,  hz],
    #     [-hx, -hy, -hz],
    # ])

    T_centroid_corners = np.tile(np.eye(4), (8, 1, 1))
    T_centroid_corners[:, :3, 3] = translations

    return T_centroid_corners


def OBB_to_AABB(T_world_corners):
    """
    Convert a world-space Oriented Bounding Box (OBB), represented by its
    8 corner transforms, into Bounds of Axis-Aligned Bounding Box (AABB).

    Args:
        T_world_corners (np.ndarray): (8, 4, 4) containing world-frame transforms
            of the OBB corners.
    Returns:
        (np.ndarray, np.ndarray): (min_xyz, max_xyz), where each is a (3,) vector in world frame.
    """
    xyz = T_world_corners[:, :3, 3]
    return xyz.min(axis=0), xyz.max(axis=0)


def check_AABB_collision(a, b) -> bool:
    """
    Check for collision between two AABBs
    Args:
        a ((np.ndarray, np.ndarray)): (min_xyz, max_xyz) bounds of the first AABB.

        b ((np.ndarray, np.ndarray)): (min_xyz, max_xyz) bounds of the second AABB.

    Returns:
        (bool): True if the AABBs overlap, False otherwise.
    """

    a_min, a_max = a
    b_min, b_max = b

    return np.all(a_min <= b_max) and np.all(a_max >= b_min)


def interpolate_transform(T0, T1, num_steps=10):
    """
    Interpolate between two SE(3) transforms.
    TODO
    Source: Mostly GPT
    """
    p0 = T0[:3, 3]
    p1 = T1[:3, 3]

    r0 = R.from_matrix(T0[:3, :3])
    r1 = R.from_matrix(T1[:3, :3])

    slerp_obj = Slerp([0, 1], R.concatenate([r0, r1]))

    Ts = []
    for t in np.linspace(0, 1, num_steps):
        p = (1 - t) * p0 + t * p1
        R_t = slerp_obj(t).as_matrix()

        T = np.eye(4)
        T[:3, :3] = R_t
        T[:3, 3] = p
        Ts.append(T)

    return Ts

                     
def check_collisions(obj:dict,  tracked_objects:dict) -> bool:
    """
    Check whether a given object collides with any other tracked object
    using AABB-based broad-phase collision detection.

    TODO: obj as object instead of array???
    """
    T_world_corners = obj["T_world_centroid"] @ obj["T_centroid_corners"]
    obj_name = obj["name"]
    AABB_bounds = OBB_to_AABB(T_world_corners)

    for comp_obj_name, comp_obj in tracked_objects.items():
        if comp_obj_name == obj_name:
            continue

        comp_T_world_corners = comp_obj["T_world_centroid"] @ comp_obj["T_centroid_corners"]
        comp_AABB_bounds = OBB_to_AABB(comp_T_world_corners)
        if check_AABB_collision(AABB_bounds, comp_AABB_bounds):
            return True, comp_obj, comp_AABB_bounds


    return False, None, None


def check_collisions_along_interpolation(obj, final_T_world_centroid, tracked_objects):
    # Init pose comes from obj
    init_T_world_centroid = obj["T_world_centroid"]

    for T in interpolate_transform(init_T_world_centroid, final_T_world_centroid, 20):
        obj_interp = copy.deepcopy(obj)
        obj_interp["T_world_centroid"] = T

        is_collided, collided_obj, collided_AABB_bounds = check_collisions(obj_interp, tracked_objects)
        if is_collided:
            return True, collided_obj, collided_AABB_bounds
    
    return False, None, None
    



# def pick(arm: str, grasp_pose: list, end_position:list, object_name: str, tracked_objects) -> list[dict]:
def pick(prim: dict, tracked_objects=None, run_checks=True) -> list[dict]:
    """
    Go to object, envelop_grasp, and translate. Keep same orientation after grasping.
    Args:
        arm (str): Which arm to use. Options: 'left', 'right'.
        grasp_pose (list): (7,) Pose to grasp the object at in m/rad [x,y,z,qx,qy,qz,qw].
        end_position (list): (3,) Position to move object to in m [x,y,z].
            If None, does not move object.
        object_name (str): Name of object being picked.
        tracked_objects: TODO
    Returns:
        (list[dict]): Array of core primitive dicts that make up prim.
    """

    params = prim["parameters"]
    arm = params["arm"]
    end_position = params["end_position"]
    grasp_pose = params["grasp_pose"]

    object_name = params.get("object")
    obj = tracked_objects.get(object_name)

    ####################### OBJECT PARAMETER CHECKING #######################
    if obj and run_checks:
    
        # Overwrite  grasp_pose with more accurate grasp_pose based on object
        T_centroid_grasp = obj["T_centroid_grasp"]
        T_world_centroid = obj["T_world_centroid"]
        T_world_grasp = T_world_centroid @ T_centroid_grasp
        
        # Round to 2 decimals for better display
        T_world_grasp = np.around(T_world_grasp, 2)
        grasp_pose = list(transformation_to_pose(T_world_grasp))
        params["grasp_pose"] =  grasp_pose


    ##########################################################################
    
    
    pre_grasp_pose = grasp_pose.copy()
    pre_grasp_pose[2] += 0.05 # 5 cm above

    
    core_prims = [
        {'name': 'move_to_pose',
         'parameters': {
             'arm': arm,
             'pose': pre_grasp_pose},
         'core_primitives': None},
        {'name': 'move_to_pose',
         'parameters': {
             'arm': arm,
             'pose': grasp_pose},
         'core_primitives': None},
        {'name': 'envelop_grasp',
         'parameters': {
             'arm': arm,
             'object': object_name},
         'core_primitives': None},
        {'name': 'move_to_pose',
         'parameters': {
             'arm': arm,
             'pose': end_position + grasp_pose[3:],
             'object': object_name},
         'core_primitives': None}
    ]

    prim["params"] = params
    prim["core_primitives"] = core_prims


    return prim


def pour(prim:dict, tracked_objects:dict=None, run_checks=True) -> list[dict]:
    """
    Angle robot to pour and then return to current position.
    Args:
        arm (str): Which arm to use. Options: 'left', 'right'.
        initial_pose (list): (7,) Pose to start pour at m/rad [x,y,z,qx,qy,qz,qw].
        pour_orientation (list): (5,) Orientation to pour at [qx,qy,qz,qw].
        pour_hold (float): Seconds to hold pour.
        object_name (str): Name of object being poured.
        receiving_object_name (str): Name of container receiving pour.
        tracked_objects: TODO
    Returns:
        (list[dict]): Array of core primitive dicts that make up prim.
    """

    params = prim["parameters"]
    arm = params["arm"]
    initial_pose = params["initial_pose"]

    pour_orientation = params["pour_orientation"]
    pour_hold = params["pour_hold"]


    object_name = params.get("object")
    receiving_object_name = params.get("receiving_object_name")
    obj = tracked_objects.get(object_name)
    receiving_obj = tracked_objects.get(receiving_object_name)

    ####################### OBJECT PARAMETER CHECKING #######################
    if obj and receiving_obj and run_checks:

        # Check that the pour object is above the receiving object.
        T_world_corners = receiving_obj["T_world_centroid"] @ receiving_obj["T_centroid_corners"]

        # Highest position in receiving object (min that centroid can be)
        min_z = np.max(T_world_corners[:, 2, 3]) 
        
        T_world_grasp = pose_to_transformation(initial_pose)
        T_world_centroid = T_world_grasp  @ np.linalg.inv(obj["T_centroid_grasp"])
        z = T_world_centroid[2, 3]

        if z <= min_z:

            initial_pose[2] = min_z
            params["initial_pose"] =  initial_pose


    ##########################################################################

    # TODO: IMPLEMENT WAIT

    core_prims = [
        {'name': 'move_to_pose',
         'parameters': {
             'arm': arm,
             'pose': initial_pose,
             'object': object_name},
         'core_primitives': None},
        {'name': 'move_to_pose',
         'parameters': {
             'arm': arm,
             'pose': initial_pose[:3] +  pour_orientation,
             'object': object_name},
         'core_primitives': None},
        # {'name': 'wait',
        # 'parameters': {
        #     'arm': arm,
        #     'seconds': pour_hold },
        #  'core_primitives': None},
        {'name': 'move_to_pose',
         'parameters': {
             'arm': arm,
             'pose': initial_pose,
             'object': object_name},
         'core_primitives': None},
    ]


    prim["params"] = params
    prim["core_primitives"] = core_prims


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
            {'name': 'prim_name', parameters: {'arm': 'left', 'pose': [0,0,0,0,0,0,1], 'joint_state': (['joint_1',...], [0.1,...])}
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

        joint_state = params.get("joint_state")
        if joint_state is not None:
            joint_state_msg = JointState()
            joint_state_msg.name = joint_state[0]
            joint_state_msg.position = joint_state[1]
            prim_msg.joint_state = joint_state_msg

    return prim_msg


def prim_plan_to_ros_msg(flattened_prim_plan:list[dict]) -> list[PrimitiveMsg]:
    """
    Take in the flattened core primitives and turn into a ROS
    message list that can be sent to primitive_action_handler_node.
    Args:
        flattened_prim_plan (list[dict]): List of core primitives in the form of
            [{'name': 'prim_name', parameters: {'arm': 'left', 'pose': [0,0,0,0,0,0,1], 'joint_state': (['joint_1',...], [0.1,...])}]
    Returns:
        (list[PrimitiveMsg]): List of ROS Messages.
    """
    primitive_list = []
    for prim in flattened_prim_plan:
        primitive_list.append(core_prim_to_ros_msg(prim))
    return primitive_list



