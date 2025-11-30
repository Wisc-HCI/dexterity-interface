
from primitive_msgs_ros.action import Primitives
from robot_motion_interface_ros_msgs.action import  SetJointPositions, SetCartesianPose

import time
import numpy as np
import rclpy
import threading
from rclpy.action.server import ServerGoalHandle
from action_msgs.msg import GoalStatus
from rclpy.action import ActionServer, CancelResponse, ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Empty, String

# TODO: Implement this for Teleop too (so no duplication)

class PrimitiveActionHandlerNode(Node):

    def __init__(self):
        """
        Allows you to send a series of primitive action goals to the robot. These goals are blocking.
        This node is a hardcoded with positions/joints specific to the bimanual panda system
        """
        super().__init__('primitive_action_handler_node')
        
        #################### Parameters ####################

        # Primitive actions
        self.declare_parameter('primitive_action', '/primitives')

        # # Interface action customization
        # self.declare_parameter('set_joint_state_action', '/set_joint_state')
        self.declare_parameter('set_cartesian_pose_action', '/set_cartesian_pose')

        # # Primitive actions
        primitive_action = self.get_parameter('primitive_action').value


        # # Interface action customization
        # set_joint_state_action = self.get_parameter('set_joint_state_action').value
        set_cartesian_pose_action = self.get_parameter('set_cartesian_pose_action').value
        

        #################### Action Server ####################

        # Only allow one action at at time
        self._primitive_goal_lock = threading.Lock()
        self._primitive_goal_handle = None

        self._primitive_action_server = ActionServer(
            self, Primitives, primitive_action,
            execute_callback=self.primitive_execute_callback,
            # handle_accepted_callback=self.primitive_handle_accepted_callback,
            cancel_callback=self.primitive_cancel_callback,
            callback_group=ReentrantCallbackGroup())
        
        self._cart_pose_action_client = ActionClient(self, SetCartesianPose, set_cartesian_pose_action)

    ##################### Primitives #####################
    def move_to_pose(self, arm:str, pose:PoseStamped, goal_handle: ServerGoalHandle) -> bool:
        """
        Moves arm to specified pose

        TODO: COMPLETE
        Args:
            pose (PoseStamped): Requires 'right' or 'left' at msg.header.frame_id and
                pose at msg.pose.position.x/y/z (m) and msg.pose.orientation.x/y/z/w (quat). 

        Returns:
            (bool): True if succeeded, else False.
        """
        if arm == 'left':
            pose.header.frame_id = 'left_delto_offset_link'
        elif arm == 'right':
            pose.header.frame_id = 'right_delto_offset_link'
        
        
        # TODO: HANDLE INTERUPT
        self.get_logger().info('Waiting for action server...')
        self._cart_pose_action_client.wait_for_server()
        self.get_logger().info('Connected to action server.')

        goal_msg = SetCartesianPose.Goal()
        goal_msg.pose_stamped = pose

        goal_future = self._cart_pose_action_client.send_goal_async(goal_msg)


        # Wait for goal to be recieved
        while not goal_future.done():
            if goal_handle.is_cancel_requested:
                self.get_logger().warn('Primitive Action Cancelled so move_to_pose() cancelled')
                return False

            time.sleep(0.05)

        move_goal_handle = goal_future.result()
        if not move_goal_handle.accepted:
            self.get_logger().warn('Goal rejected :(')
            return False


        result_future = move_goal_handle.get_result_async()

        while not result_future.done():
            if goal_handle.is_cancel_requested:
                self.get_logger().warn('Primitive Action Cancelled so move_to_pose() cancelled')
                move_goal_handle.cancel_goal_async()
                return False

            time.sleep(0.05)

        status = result_future.result().status
        if status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().warn('Goal failed!')

        return True

    
    

    # def envelop_grasp(self, msg: String):
    #     """
    #     Subscriber callback for grasping with enveloping grasp
    #     Args:
    #         msg (String): String with either 'right' or 'left' depending on arm
    #     """
    #     arm = msg.data

    #     pub_msg = JointState()
    #     pub_msg.position = [1.5, 1.5, 1.5, 1.5, 1.5, 1.5]
    #     if arm == 'left':
    #         pub_msg.name = ["left_F1M3", "left_F1M4", "left_F2M3", "left_F2M4", "left_F3M3", "left_F3M4"]
    #     elif arm == 'right':
    #         pub_msg.name = ["right_F1M3", "right_F1M4", "right_F2M3", "right_F2M4", "right_F3M3", "right_F3M4"]
        
    #     else:
    #         self.get_logger().error(f"ERROR in envelop_grasp: {arm} not an option for msg.data. Options: 'right', 'left'.")

    #     self._set_joint_state_publisher.publish(pub_msg)


    # def release(self, msg: String):
    #     """
    #     Subscriber callback for openeing the gripper
    #     Args:
    #         msg (String): String with either 'right' or 'left' depending on arm
    #     """
    #     arm = msg.data

    #     pub_msg = JointState()
    #     # Release full
    #     # pub_msg.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    #     # Release with fingers parallel
    #     pub_msg.position = [0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0]
    #     if arm == 'left':
    #         pub_msg.name = ["left_F1M1", "left_F1M2", "left_F1M3", "left_F1M4", 
    #                         "left_F2M1", "left_F2M2", "left_F2M3", "left_F2M4", 
    #                         "left_F3M1", "left_F3M2", "left_F3M3", "left_F3M4"]

    #     elif arm == 'right':
    #         pub_msg.name = ["right_F1M1", "right_F1M2", "right_F1M3", "right_F1M4", 
    #                         "right_F2M1", "right_F2M2", "right_F2M3", "right_F2M4", 
    #                         "right_F3M1", "right_F3M2", "right_F3M3", "right_F3M4"]
    #     else:
    #         self.get_logger().error(f"ERROR in envelop_grasp: {arm} not an option for msg.data. Options: 'right', 'left'.")

    #     self._set_joint_state_publisher.publish(pub_msg)

    ##################### Action Server #####################


    def primitive_cancel_callback(self, goal_handle: ServerGoalHandle) -> CancelResponse:
            """
            Accept client request to cancel an action.
            Args:
                goal_handle (ServerGoalHandle): Client goal handler (unused).
            Returns
                (CancelResponse): Always accepts the cancellation
            """
            self.get_logger().info('Received cancel request.')
            return CancelResponse.ACCEPT
    
    
    # def primitive_handle_accepted_callback(self, goal_handle: ServerGoalHandle):
    #     """
    #     Handles any primitive goal once accepted (Home, SetCartesianPose, SetJointPositions
    #     and aborts previous goal (if applicable).
    #     Args:
    #         goal_handle (ServerGoalHandle): Client goal handler. 
    #     """

    #     with self._primitive_goal_lock:
    #         # This server only allows one goal at a time
    #         if self._primitive_goal_handle is not None and self._primitive_goal_handle.is_active:
    #             self.get_logger().info('Aborting previous goal')
    #             # Abort the existing goal
    #             self._primitive_goal_handle.abort()
    #         self._primitive_goal_handle = goal_handle

    #     goal_handle.execute()

    def primitive_execute_callback(self, goal_handle: ServerGoalHandle) -> Primitives.Result:
        """
        Executes the list of primitives passed.
        Args:
            goal_handle (ServerGoalHandle): Client goal handler.
        Returns:
            (Primitives.Result): success set to True if the robot executes all the primitives, and
            False if the action is canceled or fails.
        """

        primitives = goal_handle.request.primitives

        feedback = Primitives.Feedback()
        result = Primitives.Result()
        for i, prim in enumerate(primitives):

            if goal_handle.is_cancel_requested:
                self.get_logger().info('CANCEL REQUESTED')
                # TODO: HANDLE AT LOWER LEVEL
                result.success = False
                result.final_idx = i
                goal_handle.canceled()
                return result

            type = prim.type
            arm = prim.arm
            pose = prim.pose


            feedback.current_idx = i
            goal_handle.publish_feedback(feedback)

            if arm != 'left' and arm != 'right':
                self.get_logger().error(f" Primitive arm {arm} not supported. Options: 'right', 'left'.")
                result.success = False
                result.final_idx = i
                return result

            succeeded = False
            if type == "move_to_pose":
                succeeded = self.move_to_pose(arm, pose, goal_handle)
            else:
                self.get_logger().warn(f"Primitive type {type} is not supported. Options: 'move_to_pose'")
                succeeded = False
            
            if not succeeded:
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                result.success = False
                result.final_idx = i
                return result
        
        goal_handle.succeed()
        result.success = True
        result.final_idx = i
        return result
        
def main(args=None):
    rclpy.init(args=args)
    node = PrimitiveActionHandlerNode()
    executor = MultiThreadedExecutor()
    try:
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()