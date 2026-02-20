from primitives_ros.utils.transformation_utils import pose_to_transformation, transformation_to_pose
from primitive_msgs_ros.msg import Primitive as PrimitiveMsg

import copy

from scipy.spatial.transform import Rotation as R, Slerp

import numpy as np

from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped


CORE_PRIMITIVES = {'move_to_pose', 'move_to_joint_positions', 'envelop_grasp', 'pincer_grasp', 'release', 'home'}

ARM_COLLISION_THRESHOLD = 0.35  # m — EE-to-EE distance below which a retract is triggered

# Compact safe cartesian retract pose per arm [x, y, z, qx, qy, qz, qw]
RETRACT_POSES = {
    "left":  [-0.4, 0, 1.4, 1, 0, 0, 0],
    "right": [0.4, 0, 1.4, 1, 0, 0, 0],
}

ARM_HOME_POSES = {
    "left": [-0.25, 0, 1.37, 1, 0, 0, 0], 
    "right": [0.25, 0, 1.37, 1, 0, 0, 0]
}

def _make_retract_prim(arm: str) -> dict:
    return {
        'name': 'move_to_pose',
        'parameters': {'arm': arm, 'pose': RETRACT_POSES[arm]},
        'core_primitives': None,
    }


def update_arm_tracking(core_prim: dict, tracked_arms: dict) -> dict:
    """Update simulated arm EE poses. Only move_to_pose changes tracked position."""
    params = core_prim["parameters"]
    arm = params.get("arm")
    if arm and core_prim["name"] == "move_to_pose":
        tracked_arms[arm] = params["pose"]
    return tracked_arms


def inject_arm_retracts(core_prims: list[dict], tracked_arms: dict,
                        threshold: float = ARM_COLLISION_THRESHOLD) -> tuple[list[dict], dict]:
    """
    Scan a list of core primitives and insert a retract for the opposite arm
    before any move_to_pose that would bring the two EEs within `threshold` metres.

    Args:
        core_prims: Ordered list of core primitive dicts.
        tracked_arms: {'left': pose_7d_or_None, 'right': pose_7d_or_None}
        threshold: EE-to-EE distance (m) that triggers a retract.
    Returns:
        (list[dict]): New list with retract prims injected where needed.
        (dict): Updated tracked_arms.
    """
    result = []
    for cp in core_prims:
        if cp["name"] == "move_to_pose":
            arm  = cp["parameters"].get("arm")
            pose = cp["parameters"].get("pose")
            if arm and pose is not None:
                other_arm  = "right" if arm == "left" else "left"
                other_pose = tracked_arms.get(other_arm)
                if other_pose is not None and other_arm in RETRACT_POSES:
                    dist = np.linalg.norm(np.array(pose[:3]) - np.array(other_pose[:3]))
                    if dist <= threshold:
                        print(f"[ARM COLLISION] retracting {other_arm} (EE dist={dist:.3f} m)")
                        result.append(_make_retract_prim(other_arm))
                        tracked_arms = update_arm_tracking(result[-1], tracked_arms)
        result.append(cp)
        tracked_arms = update_arm_tracking(cp, tracked_arms)
    return result, tracked_arms


def parse_prim_plan(prim_plan:list[dict], objects:list[str] = [], joint_state:dict=None,
                    initial_ee_poses:dict=None) -> list[dict]:
    """
    Takes high-level (and core) primitive plan and parses it into purely core primitives using
    envelop_grasp, release, move_to_pose, home.

    Args:
        prim_plan (list[dict]): List of high level and core primitives in form of:
            [{'name': 'envelop_grasp', parameters: {'arm': 'left', pose: [0,0,0,0,0,0,1]} }, ...]

        objects (list[dict]): List of object dictionaries with the form:  {'name': ..., 'description': ..., 'pose': ..., grasp_pose: ..., dimensions: ....}
            - pose (np.ndarray): (7,)  Centroid of object (with z at the bottom of object) in [x,y,z, qx, qy, qz, qw] in m
            - grasp_pose (np.ndarray): (7,) Pose to grasp relative to centroid [x,y,z, qx, qy, qz, qw] in m.
            - dimensions (np.ndarray): (3,) [x (width), y (length), z (height)] in m

        joint_state (dict): Current joint positions per arm: {"left": [7 floats], "right": [7 floats]}.
            If None, cuRobo trajectory planning is skipped.

        initial_ee_poses (dict): Current EE pose per arm {"left": [x,y,z,qx,qy,qz,qw], "right": ...}.
            Used to seed arm collision checking from the start of the plan.
            If None, falls back to RETRACT_POSES.

    Returns:
        (list[dict]): List of primitives with the high-level primitives containing
            all the core primitives in form of [{'name': 'envelop_grasp', parameters: {'arm': 'left', pose: [0,0,0,0,0,0,1]}, core_primitives: {...} }, ...]
    """

    parsed_plan = []

    objects = copy.deepcopy(objects)
    # TODO: ADD THIS ELSWHERE
    objects.append({
        "name": "table",
        "pose": np.array([0.0, 0.0, 0.9144, 0, 0, 0, 1]),
        "grasp_pose": np.array([0, 0, 0, 0, 0, 0, 1]),  # unused but needed by object_list_to_dict
        "dimensions": np.array([1.8288, 0.62865, 0.045])
    })

    tracked_objects = object_list_to_dict(objects)
    tracked_arms = {
        "left":  (initial_ee_poses or {}).get("left",  ARM_HOME_POSES["left"]),
        "right": (initial_ee_poses or {}).get("right", ARM_HOME_POSES["right"]),
    }

    for prim in prim_plan:
        name = prim.get('name')
        if name and name in CORE_PRIMITIVES:
            prim['core_primitives'] = None
            prim, _ = repair_core_primitive(prim, tracked_objects)
            tracked_objects = update_object_tracking(prim, tracked_objects)
            expanded, tracked_arms = inject_arm_retracts([prim], tracked_arms)
            parsed_plan.extend(expanded)
        elif name:
            params = prim.get('parameters')
            if not params:
                raise ValueError(f"Primitive '{name}' needs to have 'parameters' key")
            if name == "pick":
                prim = pick(prim, tracked_objects)
            elif name == "pick_and_place":
                prim = pick_and_place(prim, tracked_objects)
            elif name == "pour":
                prim = pour(prim, tracked_objects)
            else:
                raise ValueError(f"Primitive '{name}' is not valid.")

            prim["core_primitives"], tracked_arms = inject_arm_retracts(
                prim["core_primitives"], tracked_arms)
            for core_prim in prim["core_primitives"]:
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
    obj_name = core_prim["parameters"].get("object")
    
    # HOME affects both arms
    if prim_name == "home":
        for obj in tracked_objects.values():
            obj["grasped_by"] = None
        return tracked_objects
    
    if obj_name is None:
        return tracked_objects

    obj = compute_object_state_change(core_prim, tracked_objects[obj_name])
    
    if obj is None:
        return tracked_objects

    tracked_objects[obj_name] = obj

    return tracked_objects



def compute_object_state_change(core_prim: dict, obj: dict):
    """
    Computes the effect of a core primitive on a single object.
    Does NOT mutate anything. If nothing is updated, returns original obj
    
    Returns:
        (dict) copy of obj with the following keys adjusted:
          - grasped_by (optional)
          - T_world_centroid (optional)
    """

    if not obj:
        return obj
    
    obj = copy.deepcopy(obj)

    prim_name = core_prim["name"]
    params = core_prim["parameters"]
    prim_arm = params.get("arm")
    obj_arm = obj["grasped_by"]


    if prim_name == "envelop_grasp" or prim_name == "pincer_grasp":
        obj["grasped_by"] = prim_arm
    elif prim_arm == obj_arm:
        if prim_name == "release":
            obj["grasped_by"] = None
        elif prim_name == "move_to_pose":
            T_world_ee = pose_to_transformation(params["pose"])
            new_T_centroid_grasp = T_world_ee @ np.linalg.inv( obj["T_centroid_grasp"])
            obj["T_world_centroid"] = new_T_centroid_grasp

    return obj


def repair_core_primitive(core_prim, tracked_objects) -> tuple[dict,bool]:
    """
    Returns tuple (primitive, true_if_prim_changed)
    """
    prim_name = core_prim["name"]
    if prim_name != "move_to_pose":
        return core_prim, False

    params = core_prim["parameters"]
    obj_name = params.get("object")

    if obj_name is None:
        return core_prim, False

    obj = tracked_objects[obj_name]
    moved_obj = compute_object_state_change(core_prim, obj)

    # If object pose did not change, nothing to repair
    if np.allclose(moved_obj["T_world_centroid"], obj["T_world_centroid"], 
                   atol=1e-6 ):
        return core_prim, False

    collided, collided_obj, _ = check_collisions_along_interpolation(
        obj, moved_obj["T_world_centroid"], tracked_objects
    )

    if not collided:
        return core_prim, False

    # "Replan" by raising
    dz = calculate_minimum_clearance_height(moved_obj, collided_obj)
    new_pose = params["pose"].copy()
    new_pose[2] += dz

    print(f"[REPLAN] lifting {obj_name} by {dz:.3f} m in {prim_name}")

    new_prim = copy.deepcopy(core_prim)
    new_prim["parameters"]["pose"] = new_pose

    return new_prim, True


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
            # Bounding box of object with 8 corners, relative to centroid with 1cm buffer
            "T_centroid_corners": dimensions_to_bounding_box_transforms(obj["dimensions"], 0.01)
           
        }

        tracked_objects[name] = formatted_obj

    return tracked_objects


def calculate_minimum_clearance_height(moving_obj: dict, stationary_obj: dict, 
                                       clearance_buffer: float = 0.0) -> float:
    """
    Calculate the minimum distance adjustment needed to clear one object over another (dz).
    
    Args:
        moving_obj (dict): Object being moved (with T_world_centroid)
        stationary_obj (dict): Object to clear over
        clearance_buffer: Additional safety margin in meters
    
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

    def generate_sequence(grasp_pose, end_position, object_name):
        set_pose = end_position + grasp_pose[3:]
        pre_grasp_pose = grasp_pose.copy(); pre_grasp_pose[2] += 0.2
        pre_set_pose   = set_pose.copy();   pre_set_pose[2]   += 0.2
        

        core_prims = [
            {'name': 'move_to_pose', 'parameters': {'arm': arm, 'pose': pre_grasp_pose},                        'core_primitives': None},
            {'name': 'move_to_pose', 'parameters': {'arm': arm, 'pose': grasp_pose},                            'core_primitives': None},
            {'name': 'pincer_grasp', 'parameters': {'arm': arm,                         'object': object_name}, 'core_primitives': None},
            {'name': 'move_to_pose', 'parameters': {'arm': arm, 'pose': pre_grasp_pose, 'object': object_name}, 'core_primitives': None},
            {'name': 'move_to_pose', 'parameters': {'arm': arm, 'pose': pre_set_pose,   'object': object_name}, 'core_primitives': None},
            {'name': 'move_to_pose', 'parameters': {'arm': arm, 'pose': set_pose,       'object': object_name}, 'core_primitives': None},
        ]

        return core_prims


    params = prim["parameters"]
    arm = params["arm"]
    end_position = params["end_position"]
    grasp_pose = params["grasp_pose"]

    object_name = params.get("object")
    obj = tracked_objects.get(object_name)

    core_primitives = generate_sequence(grasp_pose, end_position, object_name)

    ####################### OBJECT PARAMETER CHECKING #######################

    # Check that grasp location is correct
    if obj and run_checks:
        
        # TODO: Add this to repair grasp???
        # Overwrite  grasp_pose with more accurate grasp_pose based on object
        T_centroid_grasp = obj["T_centroid_grasp"]
        T_world_centroid = obj["T_world_centroid"]
        T_world_grasp = T_world_centroid @ T_centroid_grasp
        
        # Round to 2 decimals for better display
        T_world_grasp = np.around(T_world_grasp, 2)
        grasp_pose = list(transformation_to_pose(T_world_grasp))
        params["grasp_pose"] =  grasp_pose

        # Regenerate based on fixes
        core_primitives = generate_sequence(grasp_pose, end_position, object_name)
    
    # Check that set is above other objects
    if tracked_objects and run_checks:
        tracked_for_set = copy.deepcopy(tracked_objects)
        SET_IDX = -1
        for cp in core_primitives[:SET_IDX]:
            tracked_for_set = update_object_tracking(cp, tracked_for_set)

        set_prim, is_changed = repair_core_primitive(core_primitives[SET_IDX], tracked_for_set)
        set_pose = set_prim["parameters"]["pose"]
        if is_changed:
            params['end_position'] = set_pose[:3]
            end_position = set_pose[:3]
            core_primitives = generate_sequence(grasp_pose, end_position, object_name)

    prim["core_primitives"] = core_primitives


    return prim


def pick_and_place(prim: dict, tracked_objects=None, run_checks=True) -> list[dict]:
    """
    Pick up object and place it at end_position then release.
    Calls pick to grasp and lift, then moves above release position,
    lowers to release, releases, and lifts away.
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
    object_name = params.get("object")

    def generate_sequence(prim, tracked_objects, run_checks):
         # Use pick to handle grasp + lift
        pick_prim = pick(copy.deepcopy(prim), tracked_objects, run_checks)
        core_prims = list(pick_prim["core_primitives"])


        lift_after_set = copy.deepcopy(core_prims[-2])
        lift_after_set['parameters'].pop('object', None)

        core_prims.append({ 'name': 'release', 'parameters': {  'arm': arm,  'object': object_name}, 'core_primitives': None})
        core_prims.append(lift_after_set)

        parameters = pick_prim["parameters"]

        return core_prims, parameters


    core_prims, parameters = generate_sequence(prim, tracked_objects, run_checks)


    prim["parameters"] = parameters
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



