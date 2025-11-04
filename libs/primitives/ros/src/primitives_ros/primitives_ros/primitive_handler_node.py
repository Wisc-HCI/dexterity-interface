import numpy as np
import rclpy
import threading
from rclpy.executors import ExternalShutdownException
from rclpy.parameter import Parameter
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Empty, String



class PrimitiveHandlerNode(Node):

    def __init__(self):
        """
        Allows you to send primitive commands to your desired interface. This node is 
        a hardcoded with positions/joints specific to the bimanual panda system
        """
        super().__init__('primitive_handler_node')
        
        #################### Parameters ####################

        # Primitive topics
        self.declare_parameter('primitive_envelop_grasp_topic', 'primitive/envelop_grasp')
        self.declare_parameter('primitive_release_topic', '/primitive/release')
        self.declare_parameter('primitive_move_to_pose_topic', 'primitive/move_to_pose')

        # Interface topic customization
        self.declare_parameter('set_joint_state_topic', 'set_joint_state')
        self.declare_parameter('set_cartesian_pose_topic', 'set_cartesian_pose')
        self.declare_parameter('home_topic', 'home')

        # Primitive topics
        primitive_envelop_grasp_topic = self.get_parameter('primitive_envelop_grasp_topic').value
        primitive_release_topic = self.get_parameter('primitive_release_topic').value
        primitive_move_to_pose_topic = self.get_parameter('primitive_move_to_pose_topic').value

        # Interface topic customization
        set_joint_state_topic = self.get_parameter('set_joint_state_topic').value
        set_cartesian_pose_topic = self.get_parameter('set_cartesian_pose_topic').value
        

        #################### Subscribers ####################
        self.create_subscription(String, primitive_envelop_grasp_topic, self.envelop_grasp, 10)
        self.create_subscription(String, primitive_release_topic, self.release, 10)
        self.create_subscription(PoseStamped, primitive_move_to_pose_topic, self.move_to_pose, 10)


        #################### Publishers ####################
        self._set_joint_state_publisher = self.create_publisher(JointState, set_joint_state_topic, 10)
        self._set_cartesian_pose_publisher = self.create_publisher(PoseStamped, set_cartesian_pose_topic, 10)


    
    def move_to_pose(self, msg:PoseStamped):
        """
        Subscriber callback function for executing move_to_pose command

        Args:
            msg (PoseStamped): Requires 'right' or 'left' at msg.header.frame_id and
                pose at msg.pose.position.x/y/z (m) and msg.pose.orientation.x/y/z/w (quat). 
        """
        pub_msg = msg
        arm = pub_msg.header.frame_id
        if arm == 'left':
            pub_msg.header.frame_id = 'left_delto_offset_link'
        elif arm == 'right':
            pub_msg.header.frame_id = 'right_delto_offset_link'
        else:
            self.get_logger().error(f"ERROR in move_to_pose: {arm} not an option for msg.header.frame_id. Options: 'right', 'left'.")

        self._set_cartesian_pose_publisher.publish(msg)

        
    def envelop_grasp(self, msg: String):
        """
        Subscriber callback for grasping with enveloping grasp
        Args:
            msg (String): String with either 'right' or 'left' depending on arm
        """
        arm = msg.data

        pub_msg = JointState()
        pub_msg.position = [1.5, 1.5, 1.5, 1.5, 1.5, 1.5]
        if arm == 'left':
            pub_msg.name = ["left_F1M3", "left_F1M4", "left_F2M3", "left_F2M4", "left_F3M3", "left_F3M4"]
        elif arm == 'right':
            pub_msg.name = ["right_F1M3", "right_F1M4", "right_F2M3", "right_F2M4", "right_F3M3", "right_F3M4"]
        
        else:
            self.get_logger().error(f"ERROR in envelop_grasp: {arm} not an option for msg.data. Options: 'right', 'left'.")

        self._set_joint_state_publisher.publish(pub_msg)


    def release(self, msg: String):
        """
        Subscriber callback for openeing the gripper
        Args:
            msg (String): String with either 'right' or 'left' depending on arm
        """
        arm = msg.data

        pub_msg = JointState()
        pub_msg.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        if arm == 'left':
            pub_msg.name = ["left_F1M3", "left_F1M4", "left_F2M3", "left_F2M4", "left_F3M3", "left_F3M4"]
        elif arm == 'right':
            pub_msg.name = ["right_F1M3", "right_F1M4", "right_F2M3", "right_F2M4", "right_F3M3", "right_F3M4"]
        
        else:
            self.get_logger().error(f"ERROR in envelop_grasp: {arm} not an option for msg.data. Options: 'right', 'left'.")

        self._set_joint_state_publisher.publish(pub_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PrimitiveHandlerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()