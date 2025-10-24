import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
import numpy as np
import os

from ranged_ik import RangedIK  # adjust if the import path differs


class RangedIKNode(Node):
    def __init__(self):
        super().__init__("ranged_ik_node")

        # Path to your settings
        settings_path = os.path.join(
            os.path.dirname(__file__), "settings.yaml"
        )
        self.rik = RangedIK(settings_path=settings_path)

        # Subscriber: listen to target poses (e.g. from RViz)
        self.pose_sub = self.create_subscription(
            PoseStamped,
            "target_pose",
            self.pose_callback,
            10,
        )

        # Publisher: publish joint states
        self.joint_pub = self.create_publisher(JointState, "joint_states", 10)

        self.get_logger().info("RangedIK ROS2 node initialized.")

    def pose_callback(self, msg: PoseStamped):
        """Handle incoming target pose messages."""
        pos = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        quat = np.array([
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        ])

        target = np.concatenate((pos, quat))
        self.get_logger().info(f"Received target pose: {target}")

        # Solve IK
        joint_angles = self.rik.solve(target, base_frame="base_link", ee_frame="ee_link")

        # Publish result as JointState
        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.name = [f"joint_{i+1}" for i in range(len(joint_angles))]
        joint_msg.position = joint_angles.tolist()
        self.joint_pub.publish(joint_msg)


def main(args=None):
    rclpy.init(args=args)
    node = RangedIKNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
