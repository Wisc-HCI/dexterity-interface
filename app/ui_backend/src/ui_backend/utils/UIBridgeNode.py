from primitives_ros.utils.create_high_level_prims import prim_plan_to_ros_msg, flatten_hierarchical_prims


import threading
import asyncio
import numpy as np
# ROS
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.action import ActionClient
from rclpy.node import Node
from primitive_msgs_ros.action import Primitives as PrimitivesAction

from robot_motion_interface_ros_msgs.msg import ObjectPoses
from geometry_msgs.msg import PoseStamped  
from sensor_msgs.msg import JointState
from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException

from ui_backend.utils.helpers import get_current_scene

class RosRunner:
    def __init__(self):
        """
        Initializes the ROS runtime and prepares an executor for spinning nodes
        as MultiThreadedExecutor.
        """
        if not rclpy.ok():
            rclpy.init()

        self._node = None
        self._executor = MultiThreadedExecutor()
        
        self._thread = None

    def start(self, node):
        """
        Starts spinning the given ROS node in a background thread.
        Args:
            node (rclpy.node.Node): The ROS node to run.
        Raises:
            RuntimeError: If a node is already being managed by this runner.
        """

        if self._node:
            raise RuntimeError("Node already started. Cannot start another.")
        
        self._node = node
        self._executor.add_node(node)
        
        self._thread = threading.Thread(
            target=self._spin,
            daemon=True
        )
        self._thread.start()
    

    def stop(self):
        """
        Stops the executor, destroys the managed node, and shuts down rclpy.
        """
        self._executor.shutdown()

        if self._node:
            self._node.destroy_node()
            self._node = None
        rclpy.try_shutdown()

    def _spin(self):
        """
        Spin the node.
        """
        try:
            self._executor.spin()
        except ExternalShutdownException:
            # Expected during shutdown
            pass

# TODO: RENAME THIS???
class UIBridgeNode(Node):
    def __init__(self):
        """
    
        Initializes ros node that acts as bridge between UI and ROS logic.
        """
        super().__init__('backend_primitive_client')
        self._sim_client = ActionClient(self, PrimitivesAction, '/primitives')
        self._real_client = ActionClient(self, PrimitivesAction, '/primitives/real')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=200, 
        )

        self._spawn_obj_pub = self.create_publisher(PoseStamped, "/spawn_object", qos)
        self._move_obj_pub = self.create_publisher(PoseStamped, "/move_object", qos)
        
        self._reset_sim_joint_state_pub = self.create_publisher(
            JointState, '/reset_sim_joint_position', 10) # For resetting arm
        self.create_subscription(ObjectPoses, "/object_poses", 
                                 self._object_poses_callback, qos)
        self.create_subscription(JointState, "/joint_state", 
                            self._joint_state_callback, 10)
        

        # Subscriber variable storage
        # Latest state cache ([{name:'', pose:''}, {}])
        self._object_poses = []
        # ([joint_names], [joint_positions])
        self._joint_state = set() 

        # Storage for Robot and scene state after each primitive is executed
        # Example for state after 1st idx primitive: {1: {joint_state: [()], object_poses: []}}
        self._primitive_scene_state = {}

        self._cur_executing_flat_idx = None
        self._goal_handle = None

        self._scene = get_current_scene(True)
        self._flat_to_hierach_idx_map = None
        self._hierach_to_flat_idx_map = None
        self._flat_start_idx = None


    def spawn_objects(self):
        """
        Initialize objects in the scene
        """
        for obj in self._scene:

            self.spawn_object(obj["name"], obj["pose"])


    def get_joint_state(self) -> dict:
        """
        Returns current joint positions for each arm.
        Returns:
            (dict): {"left": [7 floats], "right": [7 floats]}
        """

        if not self._joint_state:
            return None

        names, positions = self._joint_state
        positions = list(positions)

        left_q = []
        right_q = []
        for name, pos in zip(names, positions):
            if name.startswith("left_panda_joint"):
                left_q.append((name, pos))
            elif name.startswith("right_panda_joint"):
                right_q.append((name, pos))

        # Sort by joint number and extract positions
        left_q.sort(key=lambda x: x[0])
        right_q.sort(key=lambda x: x[0])

        return {
            "left": [p for _, p in left_q] if left_q else None,
            "right": [p for _, p in right_q] if right_q else None,
        }

    def get_scene(self, all_objects:bool=False):
        """
        Gets current scene
        Args:
            all_objects (bool): True if return all objects in the scene. False if only 
                return the major objects (used for LLM), not the minor/filling ones.
        Returns:
        (list[dict]): List of object dictionaries with the form:  {'name': ..., 'description': ..., 'pose': ..., grasp_pose: ..., dimensions: ....}
            - pose (np.ndarray): (7,)  Centroid of object (with z at the bottom of object) in [x,y,z, qx, qy, qz, qw] in m
            - grasp_pose (np.ndarray): (7,) Pose to grasp relative to centroid [x,y,z, qx, qy, qz, qw] in m.
          

        """
        return get_current_scene(all_objects)
    

    def reset_primitive_scene(self, prim_idx:list):
        """
        Restores the scene to the recorded state at the start of the given primitive in the 
        flattened plan was executed. Restores both objects and joint positions.

        The scene state must have been previously recorded for the given
        primitive index. If no state exists, the function exits without
        making any changes.

        Args:
            prim_idx (list): HIERARCHICAL index of the primitive in the
                plan whose post-execution scene state should be restored.
        """
        if not self._hierach_to_flat_idx_map:
            return
        
        flat_prim_idx = self._hierach_to_flat_idx_map[tuple(prim_idx)]
        
        self._reset_primitive_scene_flat(flat_prim_idx)
 

    def _reset_primitive_scene_flat(self, flattened_prim_idx:int):
        """
        Helper to restore the scene to the recorded state at the start of the given primitive in the 
        flattened plan was executed. If idx is 0, just resets objects, not robot position.

        Args:
            flattened_prim_idx (int): Index of the primitive in the FLATTENED
                plan whose post-execution scene state should be restored.
        """
        
        if flattened_prim_idx not in self._primitive_scene_state:
            return 
        
        prim_scene = self._primitive_scene_state[flattened_prim_idx]
        joint_names, joint_positions = prim_scene['joint_state']
        self._reset_sim_joint_positions(joint_names, joint_positions)
        self.move_objects(prim_scene['object_poses'])


    def trigger_primitives(self, primitives:list[dict], start_index, on_real:bool):
        """
        Sends primitive actions to execute on robot.
        Args:
            primitives (list[dict]): List of hierarchical primitives dicts which each dict
                can be of format of
                {'name': 'envelop_grasp', parameters: {'arm': 'left', pose: [0,0,0,0,0,0,1]}, core_primitives: [...]}
            
            on_real (bool): True if execute on real, else False.
        """

        flattened_plan, self._flat_to_hierach_idx_map, self._hierach_to_flat_idx_map = flatten_hierarchical_prims(primitives)

        if start_index is not None and start_index != [0]:
            self._flat_start_idx = self._hierach_to_flat_idx_map[tuple(start_index)]
            flattened_plan = flattened_plan[self._flat_start_idx:]
            
            # Reset objects to where they were the last time the prim was executed
            self._reset_primitive_scene_flat(self._flat_start_idx)
        else:
            # Reset objects to initial placement
            self.move_objects(self._scene)
            self._flat_start_idx = None

        self._send_plan(flattened_plan, on_real=on_real)


    def _send_plan(self, flattened_primitives:list[dict], on_real:bool=False):
        """
        Sends primitive actions to execute on robot.
        Args:
            primitives (list[dict]): List of FLATTENED core primitives dicts which each dict
                can be of format of: {'name': 'envelop_grasp', parameters: {'arm': 'left', pose: [0,0,0,0,0,0,1]}}
            on_real (bool): True if execute on real, else False.
        """
        
        goal_msg = PrimitivesAction.Goal()
        goal_msg.primitives = prim_plan_to_ros_msg(flattened_primitives)
        
        client = self._real_client if on_real else self._sim_client
        client.wait_for_server()
        future = client.send_goal_async(goal_msg, feedback_callback=self._primitive_feedback_callback)
        future.add_done_callback(self._primitive_goal_response_callback)
        return future
    

    def cancel_primitives_goal(self):
        """
        Cancel the primitive plan execution.
        """
        if self._goal_handle:
            self._goal_handle.cancel_goal_async()
        self._goal_handle = None

    
    def get_cur_executing_idx(self) -> int:
        """
        Get index of current executing primitive.
        Returns:
            (list[int]): The index of the currently executing primitive in the form of [first-level-idx,sec-level-idx,...]
                based on the primitive hierarchy from the most recently posted plan to execute.
        """

        cur_flat_idx = self._cur_executing_flat_idx
        if cur_flat_idx is None:
            return None
        
        hierarchical_idx = self._flat_to_hierach_idx_map[cur_flat_idx]
        
        return hierarchical_idx
    

    def get_cur_joint_state(self) -> tuple[list[str], list[float]]:
        """
        Gets the current joint state of the robots.
        Returns:
            (list[str]): (x,) list of joint names
            (list[float]): (x,) list of joint positions (rads)

        TODO: Numpy??
        """

        return self._joint_state


    def _primitive_feedback_callback(self, feedback_msg:PrimitivesAction.Feedback):
        """
        Handle feedback that comes from trigger_primitives goal.
        Args:
            feedback_msg (PrimitivesAction.Feedback): Message with current executing idx at 
            current_idx.
        """

        feedback = feedback_msg.feedback
        feedback_idx = feedback.current_idx
        print('Received primitive index: {0}'.format(feedback_idx))

        offset_idx = self._flat_start_idx if self._flat_start_idx else 0
        
        # Primitive feedback returns index of subset of primitives (if subset of original
        # plan) so need to convert back to original plan index
        self._cur_executing_flat_idx = feedback_idx + offset_idx

        # Don't save state if primitive was just reset or first prim
        if feedback_idx == 0:
            return 
        
        # Save state of current index
        cur_object_state = self.get_object_poses()
        cur_joint_state = self.get_cur_joint_state()
        self._primitive_scene_state[self._cur_executing_flat_idx] = {'joint_state': cur_joint_state,'object_poses': cur_object_state}
        print('Saved primitive index: {0}'.format(self._cur_executing_flat_idx))

        # Save first index (usually home) after it has executed (unique case)
        if offset_idx == 0 and feedback_idx == 1:
            self._primitive_scene_state[0] = {'joint_state': cur_joint_state,'object_poses': cur_object_state}
            

    def _primitive_goal_response_callback(self, future:asyncio.Future):
        """
        Handle the response from sending a trigger_primitives action goal.

        Args:
            future (asyncio.Future): Future containing the goal handle returned
                by the action server.
        """
        self._goal_handle = future.result()

        if not self._goal_handle .accepted:
            self.get_logger().warn('Primitive Goal rejected.')
            return

        result_future = self._goal_handle .get_result_async()
        result_future.add_done_callback(self._primitive_result_callback)

        

    def _primitive_result_callback(self, future:asyncio.Future):
        """
        Handle the final result of the trigger_primitives action.

        Args:
            future (asyncio.Future): Future containing the final result of the
                action execution.
        """
        future.result().result
        self._cur_executing_flat_idx = None
        self._goal_handle = None

        print('Finished Plan Execution. Final Received feedback: {0}'.format(self._cur_executing_flat_idx))
 

    def _joint_state_callback(self, msg: JointState):
        """
        Callback receiving the joint state
        Args:
            msg (JointState): ROS message
        """

        self._joint_state = (msg.name, msg.position)


    def _reset_sim_joint_positions(self, joint_names:list[str], joint_positions:list[float]):
        """
        Reset joint state in simulation (outside of control loop)

        Args:
            joint_names (list[str]): Names of the joints to reset.
            joint_positions (list[float]): Target joint positions (rads).
        """

        msg = JointState()

        # Fill out the JointState message
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = joint_names
        msg.position = joint_positions

        self._reset_sim_joint_state_pub.publish(msg)



    ######################## OBJECTS ########################

    def spawn_object(self, object_handle: str, pose:np.ndarray|list):
        """
        Publishes a request to spawn an object in Isaacsim at the given pose.

        Args:
            object_handle (str): Unique identifier of the object.
            pose (list): (7,) Object pose as [x, y, z, qx, qy, qz, qw].
        """


        msg = self._make_pose_stamped(object_handle, list(pose))

        self._spawn_obj_pub.publish(msg)


    def move_object(self, object_handle: str, pose:list):
        """
        Publishes a request to move an existing object to a new pose.

        Args:
            object_handle (str): Unique identifier or frame name of the object.
            pose (list): (7,) Target pose as [x, y, z, qx, qy, qz, qw] in m, rad.
        """

        msg = self._make_pose_stamped(object_handle, pose)
        self._move_obj_pub.publish(msg)

    
    def move_objects(self, objects:dict[str, float]):
        """
        Move multiple objects all at once.
        Args:
            objects (list[dict]): List of object dicts in form of
                {'name': '', pose: [x, y, z, qx, qy, qz, qw]}
                with pose in m, rad.
        """
        for obj in objects:
            self.move_object(obj['name'], obj['pose'])


    def get_object_poses(self) -> list[dict]:
        """
        Gets the current object positions in the isaacsim scene.
        Returns:
            (list[dict]): List of object dicts in form of
                {'name': '', pose: [x, y, z, qx, qy, qz, qw]}
                with pose in m, rad.

        TODO: Numpy??
        """

        return self._object_poses
    

    def _object_poses_callback(self, msg: ObjectPoses):
        """
        Callback receiving all object poses at once.
        Args:
            msg (ObjectPoses): ROS message
        """

        self._object_poses = []  # Clear
        

        for obj in msg.objects:
            ros_pose = obj.pose
            pose = {
                'name': obj.handle,
                'pose': [ ros_pose.position.x, ros_pose.position.y, ros_pose.position.z,
                    ros_pose.orientation.x, ros_pose.orientation.y, ros_pose.orientation.z, 
                    ros_pose.orientation.w]
            }
   
            self._object_poses.append(pose)


    def _make_pose_stamped(self, frame_id: str, pose: list) -> PoseStamped:
        """
        Constructs a PoseStamped message from a raw pose specification.

        Args:
            frame_id (str): Frame
            pose (list): (7,) Pose as (x, y, z, qx, qy, qz, qw).

        Returns:
            (PoseStamped): A populated PoseStamped message.
        """
        
        msg = PoseStamped()
        msg.header.frame_id = frame_id

        msg.pose.position.x = float(pose[0])
        msg.pose.position.y = float(pose[1])
        msg.pose.position.z = float(pose[2])

        msg.pose.orientation.x = float(pose[3])
        msg.pose.orientation.y = float(pose[4])
        msg.pose.orientation.z = float(pose[5])
        msg.pose.orientation.w = float(pose[6])

        return msg
    