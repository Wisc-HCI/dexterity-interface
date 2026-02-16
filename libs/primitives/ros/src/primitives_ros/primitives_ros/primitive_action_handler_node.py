
from primitive_msgs_ros.action import Primitives
from robot_motion_interface_ros_msgs.action import Home, SetJointPositions, SetCartesianPose

import time
import rclpy
import threading
from rclpy.action.server import ServerGoalHandle
from action_msgs.msg import GoalStatus
from rclpy.action import ActionServer, CancelResponse, ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped


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

        # Interface action customization
        self.declare_parameter('home_action', '/home')
        self.declare_parameter('set_joint_state_action', '/set_joint_positions')
        self.declare_parameter('set_cartesian_pose_action', '/set_cartesian_pose')
        
        # Primitive actions
        primitive_action = self.get_parameter('primitive_action').value

        # Interface action customization
        home_action = self.get_parameter('home_action').value
        set_joint_state_action = self.get_parameter('set_joint_state_action').value
        set_cartesian_pose_action = self.get_parameter('set_cartesian_pose_action').value
        

        #################### Action Server ####################

        # TODO: COMBINE THESE?
        self._primitive_executed_event = threading.Event()
        self._primitive_succeeded_event = threading.Event()

        self._primitive_action_server = ActionServer(
            self, Primitives, primitive_action,
            execute_callback=self._primitive_execute_callback,
            # handle_accepted_callback=self.primitive_handle_accepted_callback,
            cancel_callback=self._primitive_cancel_callback,
            callback_group=ReentrantCallbackGroup())
        
        #################### Action Clients ####################
        self._motion_goal_handle = None

        self._joint_pos_action_client = ActionClient(self, SetJointPositions, set_joint_state_action)
        self._cart_pose_action_client = ActionClient(self, SetCartesianPose, set_cartesian_pose_action)
        self._home_action_client = ActionClient(self, Home, home_action)
        

    ##################### Primitives #####################
    def _home(self):
        """
        Home both arms.
        """
        goal_msg = Home.Goal()
        
        # TODO: Add timeout
        goal_future = self._home_action_client.send_goal_async(goal_msg)
        goal_future.add_done_callback(self._goal_response_callback)


    def _move_to_pose(self, arm:str, pose:PoseStamped):
        """
        Moves arm to the specified pose.

        Args:
            arm (str): String with either 'right' or 'left' depending on arm
            pose (PoseStamped): Requires pose at msg.pose.position.x/y/z (m) 
                and pose.pose.orientation.x/y/z/w (quat). 

        """
        if arm == 'left':
            pose.header.frame_id = 'left_delto_offset_link'
        elif arm == 'right':
            pose.header.frame_id = 'right_delto_offset_link'
        
        self._cart_pose_action_client.wait_for_server()

        goal_msg = SetCartesianPose.Goal()
        goal_msg.pose_stamped = pose

        # TODO: Add timeout
        goal_future = self._cart_pose_action_client.send_goal_async(goal_msg)
        goal_future.add_done_callback(self._goal_response_callback)


    def _move_to_joint_position(self, joint_state:JointState):
        """
        Moves arm to the specified joint positions. Primarily for resetting

        Args:
            joint_state (JointState): Requires joint positions (rads) at joint_state.position
                and joint names at joint_state.name.
        """
        
        goal_msg = SetJointPositions.Goal()
        goal_msg.joint_state = joint_state
        
        # TODO: Add timeout
        goal_future = self._joint_pos_action_client.send_goal_async(goal_msg)
        goal_future.add_done_callback(self._goal_response_callback)


    def _envelop_grasp(self, arm:str,):
        """
        Grasp with enveloping grasp.

        Args:
            arm (str): String with either 'right' or 'left' depending on arm
        """

        joint_state = JointState()
        joint_state.position = [1.5, 1.5, 1.5, 1.5, 1.5, 1.5]
        if arm == 'left':
            joint_state.name = ["left_F1M3", "left_F1M4", "left_F2M3", "left_F2M4", "left_F3M3", "left_F3M4"]
        elif arm == 'right':
            joint_state.name = ["right_F1M3", "right_F1M4", "right_F2M3", "right_F2M4", "right_F3M3", "right_F3M4"]
        
        self._move_to_joint_position(joint_state)

    def _pincer_grasp(self, arm: str):
        """
        Grasp with 3 finger pincer grasp. Good for grasps from the top of objects.

        Args:
            arm (str): String with either 'right' or 'left' depending on arm
        """

        joint_state = JointState()
        joint_state.position = [ 0.0, 0.0, 2.0, 0.2,
                                -1.0, 0.0, 2.0, 0.2,
                                 1.0, 0.0, 2.0, 0.2]
        if arm == 'left':
            joint_state.name = ["left_F1M1", "left_F1M2", "left_F1M3", "left_F1M4", 
                            "left_F2M1", "left_F2M2", "left_F2M3", "left_F2M4", 
                            "left_F3M1", "left_F3M2", "left_F3M3", "left_F3M4"]
            
        elif arm == 'right':
            joint_state.name = ["right_F1M1", "right_F1M2", "right_F1M3", "right_F1M4", 
                            "right_F2M1", "right_F2M2", "right_F2M3", "right_F2M4", 
                            "right_F3M1", "right_F3M2", "right_F3M3", "right_F3M4"]
        
        self._move_to_joint_position(joint_state)
        

    def _release(self, arm: str):
        """
        Open the gripper with enveloping grasp.

        Args:
            arm (str): String with either 'right' or 'left' depending on arm
        """

        joint_state = JointState()

        # Release full
        # joint_state.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        # Release with fingers parallel
        joint_state.position = [0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0]
        if arm == 'left':
            joint_state.name = ["left_F1M1", "left_F1M2", "left_F1M3", "left_F1M4", 
                            "left_F2M1", "left_F2M2", "left_F2M3", "left_F2M4", 
                            "left_F3M1", "left_F3M2", "left_F3M3", "left_F3M4"]
            
        elif arm == 'right':
            joint_state.name = ["right_F1M1", "right_F1M2", "right_F1M3", "right_F1M4", 
                            "right_F2M1", "right_F2M2", "right_F2M3", "right_F2M4", 
                            "right_F3M1", "right_F3M2", "right_F3M3", "right_F3M4"]

        self._move_to_joint_position(joint_state)


    ##################### Action Client Helpers #####################

    def _goal_response_callback(self, future):
        """
        Callback invoked when the action server responds to a sent motion goal.

        Args:
            future (rclpy.task.Future): Future containing the goal handle returned
                by the action server.
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Motion goal rejected.')
            return

        self._motion_goal_handle =  goal_handle

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    def _result_callback(self, future):
        """
        Callback invoked when the action server finishes executing the motion goal.

        Args:
            future (rclpy.task.Future): Future containing the result of the executed
                motion goal, including its final status.
        """
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self._primitive_succeeded_event.set()
        else:
            self.get_logger().error('Motion goal failed.')

        self._motion_goal_handle = None
        self._primitive_executed_event.set()
    



    ##################### Action Server #####################


    def _primitive_cancel_callback(self, goal_handle: ServerGoalHandle) -> CancelResponse:
            """
            Accept client request to cancel an action.
            Args:
                goal_handle (ServerGoalHandle): Client goal handler (unused).
            Returns
                (CancelResponse): Always accepts the cancellation
            """
            self.get_logger().info('Received primitive cancel request.')
            return CancelResponse.ACCEPT
    

    def _fail_primitives(self, goal_handle: ServerGoalHandle, result: Primitives.Result, i: int) -> Primitives.Result:
        """
        Terminates execution of the primitive action due to failure or cancellation.

        Args:
            goal_handle (ServerGoalHandle): The server-side goal handle for the
                primitive action being executed.
            result (Primitives.Result): Result message to populate and return.
            i (int): Index of the primitive at which execution failed or was canceled.

        Returns:
            Primitives.Result: The populated result message with success set to False
                and final_idx indicating where execution stopped.
        """
        if self._motion_goal_handle is not None:
            self._motion_goal_handle.cancel_goal_async()
        result.success = False
        result.final_idx = i

        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
        else:
            goal_handle.abort()

        return result

    def _primitive_execute_callback(self, goal_handle: ServerGoalHandle) -> Primitives.Result:
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
                return self._fail_primitives(goal_handle, result, i)

            type = prim.type
            arm = prim.arm
            
            self.get_logger().info(f'Executing {type} on {arm} arm.')
            feedback.current_idx = i
            goal_handle.publish_feedback(feedback)

            # TODO: HANDLE specifically per primitive
            if arm != 'left' and arm != 'right' and arm != '':
                self.get_logger().error(f" Primitive arm '{arm}' not supported. Options: 'right', 'left'.")
                return self._fail_primitives(goal_handle, result, i)

            self._primitive_executed_event.clear()
            self._primitive_succeeded_event.clear()
            
            if type == "home":
                self._home()
            elif type == "move_to_pose":
                pose = prim.pose
                self._move_to_pose(arm, pose)
            elif type == "envelop_grasp":
                self._envelop_grasp(arm)
            elif type == "pincer_grasp":
                self._pincer_grasp(arm)
            elif type == "release":
                self._release(arm)
            elif type == "move_to_joint_positions":
                joint_state = prim.joint_state
                self._move_to_joint_position(joint_state)
            else:
                self.get_logger().error(f"Primitive type '{type}' not supported. Options: 'move_to_pose', 'envelop_grasp', 'pincer_grasp', 'release', 'home', 'move_to_joint_positions'")
                return self._fail_primitives(goal_handle, result, i)
            
            # Wait for primitive to execute
            while not self._primitive_executed_event.is_set():
                if goal_handle.is_cancel_requested:
                    return self._fail_primitives(goal_handle, result, i)
                time.sleep(0.01)
            
            if not self._primitive_succeeded_event.is_set():
                return self._fail_primitives(goal_handle, result, i)

        
        goal_handle.succeed()
        result.success = True
        result.final_idx = i

        self.get_logger().info("Finished executing primitives")
        return result
        
def main(args=None):
    """
    Start the ROS Primitive Action Handler
    """
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