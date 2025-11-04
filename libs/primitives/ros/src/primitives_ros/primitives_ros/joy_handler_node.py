import numpy as np
import rclpy
import threading
from rclpy.executors import ExternalShutdownException
from rclpy.parameter import Parameter
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Empty, String



class JoyHandler(Node):

    def __init__(self):
        """
        Allows you to use a joy node to teleop using the primitives.
        """
        super().__init__('joy_handler')
        
        #################### Parameters ####################
        
        # Node topics
        self.declare_parameter('joy_topic', '/joy')
        
        # Primitive topics
        self.declare_parameter('primitive_envelop_grasp_topic', '/primitive/envelop_grasp')
        self.declare_parameter('primitive_release_topic', '/primitive/release')
        self.declare_parameter('primitive_move_to_pose_topic', '/primitive/move_to_pose')

        # Node topics
        joy_topic = self.get_parameter('joy_topic').value

        # Primitive topics
        primitive_envelop_grasp_topic = self.get_parameter('primitive_envelop_grasp_topic').value
        primitive_release_topic = self.get_parameter('primitive_release_topic').value
        primitive_move_to_pose_topic = self.get_parameter('primitive_move_to_pose_topic').value

        

        #################### Subscribers ####################
        self.create_subscription(Joy, joy_topic, self.joy_callback,10)

        #################### Publishers ####################
        self._envelop_grasp_publisher = self.create_publisher(String, primitive_envelop_grasp_topic, 10)
        self._release_publisher = self.create_publisher(String, primitive_release_topic, 10)
        self._move_to_pose_publisher = self.create_publisher(PoseStamped, primitive_move_to_pose_topic, 10)


    def joy_callback(self, msg):
        """
        Subscriber callback for joy node.
        """
        # self.get_logger().info(f"Axes: {msg.axes}")
        # self.get_logger().info(f"Buttons: {msg.buttons}")
        ARM = 'left'

        L_BUMPER = msg.buttons[6]
        R_BUMPER = msg.buttons[7]


        # Grasp handleing
        grasp_msg = String()
        grasp_msg.data = ARM
        if L_BUMPER:
            self.get_logger().info("GRASPING")
            self._envelop_grasp_publisher.publish(grasp_msg)
        elif R_BUMPER:
            self.get_logger().info("RELEASING")
            self._release_publisher.publish(grasp_msg)


        # TODO: Make this handle both right as well



def main(args=None):
    rclpy.init(args=args)
    node = JoyHandler()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()