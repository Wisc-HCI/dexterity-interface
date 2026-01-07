from primitives_ros.utils.create_high_level_prims import prim_plan_to_ros_msg


import threading
import asyncio

# ROS
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from primitive_msgs_ros.action import Primitives as PrimitivesAction
from geometry_msgs.msg import PoseStamped    
from rclpy.executors import SingleThreadedExecutor, ExternalShutdownException


class RosRunner:
    def __init__(self):
        """
        Initializes the ROS runtime and prepares an executor for spinning nodes
        as SingleThreadedExecutor.
        """
        if not rclpy.ok():
            rclpy.init()

        self._node = None
        self._executor = SingleThreadedExecutor()
        
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


class UIBridgeNode(Node):
    def __init__(self):
        """
        Initializes ros node that acts as bridge between UI and ROS logic.
        """
        super().__init__('backend_primitive_client')
        self._sim_client = ActionClient(self, PrimitivesAction, '/primitives')
        self._real_client = ActionClient(self, PrimitivesAction, '/primitives/real')

        self._spawn_obj_pub = self.create_publisher(PoseStamped, "/spawn_object", 10)
        self._move_obj_pub = self.create_publisher(PoseStamped, "/move_object", 10)

        self._current_executing_idx = None


    def trigger_primitives(self, primitives:list[dict], on_real:bool=False):
        """
        Sends primitive actions to execute on robot.
        Args:
            primitives (list[dict]): List of primitives dicts which each dict
                can be of format of:
                Example: {'name': 'envelop_grasp', parameters: {'arm': 'left', pose: [0,0,0,0,0,0,1]}, core_primitives: {...} }
            on_real (bool): True if execute on real, else False.
        """
        
        goal_msg = PrimitivesAction.Goal()
        goal_msg.primitives = prim_plan_to_ros_msg(primitives)
        
        client = self._real_client if on_real else self._sim_client
        client.wait_for_server()
        future = client.send_goal_async(goal_msg, feedback_callback=self._primitive_feedback_callback)
        future.add_done_callback(self._primitive_goal_response_callback)
        return future
    
    
    def get_curr_executing_idx(self) -> int:
        """
        Get index of current executing primitive.
        Returns:
            (int): index of current executing primitive in list of FLATTENED core primitives 
                (i.e. higher-level primitives are NOT part of list). Returns None if not
                currently executing.
        """
        return self._current_executing_idx
    

    def _primitive_feedback_callback(self, feedback_msg:PrimitivesAction.Feedback):
        """
        Handle feedback that comes from trigger_primitives goal.
        Args:
            feedback_msg (PrimitivesAction.Feedback): Message with current executing idx at 
            current_idx.
        """
        feedback = feedback_msg.feedback
        print('Received feedback: {0}'.format(feedback.current_idx))
        self._current_executing_idx = feedback.current_idx


    def _primitive_goal_response_callback(self, future:asyncio.Future):
        """
        Handle the response from sending a trigger_primitives action goal.

        Args:
            future (asyncio.Future): Future containing the goal handle returned
                by the action server.
        """
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().warn('Primitive Goal rejected.')
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._primitive_result_callback)

        

    def _primitive_result_callback(self, future:asyncio.Future):
        """
        Handle the final result of the trigger_primitives action.

        Args:
            future (asyncio.Future): Future containing the final result of the
                action execution.
    """
        future.result().result
        self._current_executing_idx = None

        print('Finished Plan Execution. Final Received feedback: {0}'.format(self._current_executing_idx))
 


    ######################## OBJECTS ########################

    def spawn_object(self, object_handle: str, pose:list):
        """
        Publishes a request to spawn an object in Isaacsim at the given pose.

        Args:
            object_handle (str): Unique identifier of the object.
            pose (list): (7,) Object pose as [x, y, z, qx, qy, qz, qw].
        """

        msg = self._make_pose_stamped(object_handle, pose)
        self._spawn_obj_pub.publish(msg)


    def move_object(self, object_handle: str, pose:list):
        """
        Publishes a request to move an existing object to a new pose.

        Args:
            object_handle (str): Unique identifier or frame name of the object.
            pose (list): (7,) Target pose as [x, y, z, qx, qy, qz, qw].
        """

        msg = self._make_pose_stamped(object_handle, pose)
        self._move_obj_pub.publish(msg)

      
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
    