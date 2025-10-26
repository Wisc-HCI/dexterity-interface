#!/usr/bin/env python3
import os
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

# reuse your classes
from ranged_ik import RangedIK
from robot import Robot

path_to_current_package = get_package_share_directory('robot_motion') + '/ik/'
SETTINGS = path_to_current_package + 'settings.yaml'


class RangedIKNode(Node):
    def __init__(self):
        super().__init__('ranged_ik_node')

        # Init solver + robot (for joint names order)
        self.robot = Robot(SETTINGS)
        self.solver = RangedIK(settings_path=SETTINGS)

        # Publisher the viewer expects
        self.pub = self.create_publisher(JointState, '/relaxed_ik/joint_angle_solutions', 10)

        # Optional: subscribe to incoming goal poses (PoseStamped)
        self.create_subscription(PoseStamped, '/rik/goal_pose', self.goal_cb, 10)

        # Default goal (x,y,z,qx,qy,qz,qw)
        self.goal = np.array([0.40, 0.20, 0.30, 0.0, 0.0, 0.0, 1.0], dtype=float)

        # Drive the solver at 20 Hz
        self.timer = self.create_timer(0.05, self.tick)
        self.get_logger().info(f"Using settings: {SETTINGS}")

    def goal_cb(self, msg: PoseStamped):
        self.goal = np.array([
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,
            msg.pose.orientation.x, msg.pose.orientation.y,
            msg.pose.orientation.z, msg.pose.orientation.w
        ], dtype=float)

    def tick(self):
        # Compute IK solution
        q = self.solver.solve(self.goal, base_frame=self.robot.base_links[0], ee_frame=self.robot.ee_links[0])

        # Publish as JointState with names matching your robot
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = self.robot.articulated_joint_names
        js.position = [float(a) for a in q.tolist()]
        self.pub.publish(js)

def main():
    rclpy.init()
    node = RangedIKNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
