from primitives_ros.utils.create_high_level_prims import prim_plan_to_ros_msg


import threading

# ROS
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from primitive_msgs_ros.action import Primitives as PrimitivesAction
from primitive_msgs_ros.msg import Primitive as PrimitiveMsg 
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

        self.node = None
        self.executor = SingleThreadedExecutor()
        
        self.thread = None

    def start(self, node):
        """
        Starts spinning the given ROS node in a background thread.
        Args:
            node (rclpy.node.Node): The ROS node to run.
        Raises:
            RuntimeError: If a node is already being managed by this runner.
        """

        if self.node:
            raise RuntimeError("Node already started. Cannot start another.")
        
        self.node = node
        self.executor.add_node(node)
        
        self.thread = threading.Thread(
            target=self._spin,
            daemon=True
        )
        self.thread.start()
    

    def stop(self):
        """
        Stops the executor, destroys the managed node, and shuts down rclpy.
        """
        self.executor.shutdown()

        if self.node:
            self.node.destroy_node()
            self.node = None
        rclpy.try_shutdown()

    def _spin(self):
        """
        Spin the node.
        """
        try:
            self.executor.spin()
        except ExternalShutdownException:
            # Expected during shutdown
            pass


class UIBridgeNode(Node):
    def __init__(self):
        """
        Initializes ros node that acts as bridge between UI and ROS logic.
        """
        super().__init__('backend_primitive_client')
        self.sim_client = ActionClient(self, PrimitivesAction, '/primitives')
        self.real_client = ActionClient(self, PrimitivesAction, '/primitives/real')

        self.spawn_obj_pub = self.create_publisher(PoseStamped, "/spawn_object", 10)
        self.move_obj_pub = self.create_publisher(PoseStamped, "/move_object", 10)


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
        
        client = self.real_client if on_real else self.sim_client
        client.wait_for_server()
        future = client.send_goal_async(goal_msg)
        return future
   
    ######################## OBJECTS ########################

    def spawn_object(self, object_handle: str, pose:list):
        """
        Publishes a request to spawn an object in Isaacsim at the given pose.

        Args:
            object_handle (str): Unique identifier of the object.
            pose (list): (7,) Object pose as [x, y, z, qx, qy, qz, qw].
        """

        msg = self._make_pose_stamped(object_handle, pose)
        self.spawn_obj_pub.publish(msg)


    def move_object(self, object_handle: str, pose:list):
        """
        Publishes a request to move an existing object to a new pose.

        Args:
            object_handle (str): Unique identifier or frame name of the object.
            pose (list): (7,) Target pose as [x, y, z, qx, qy, qz, qw].
        """

        msg = self._make_pose_stamped(object_handle, pose)
        self.move_obj_pub.publish(msg)

      
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
    