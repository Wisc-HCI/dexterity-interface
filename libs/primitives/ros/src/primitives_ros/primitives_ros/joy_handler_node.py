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
        TODO: Handle right arm too
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

        self.arm = 'left'    
    
        #################### Subscribers ####################
        self.create_subscription(Joy, joy_topic, self.joy_callback,10)

        #################### Publishers ####################
        self._envelop_grasp_publisher = self.create_publisher(String, primitive_envelop_grasp_topic, 10)
        self._release_publisher = self.create_publisher(String, primitive_release_topic, 10)
        self._move_to_pose_publisher = self.create_publisher(PoseStamped, primitive_move_to_pose_topic, 10)

        self.cur_right_pose = np.array([0.259, -0.092,  0.426, 0.3839,  0.9234, 0.0, 0.0])
        self.cur_left_pose = np.array([-0.259, -0.092,  0.426, 0.9234, -0.3839, 0.0, 0.0])

         

    
    def publish_pose(self, pose:np.ndarray, arm: str):
        """
        Turn the array into pose and publish it
        Args:
            pose (np.ndarray): (7,) Pose as [x, y, z, qx, qy, qz, qq] 
                in meters and quaternions
            arm (str): either right or left
        """

        msg = PoseStamped()
        msg.header.frame_id = arm
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = pose[0]
        msg.pose.position.y = pose[1]
        msg.pose.position.z = pose[2]
        
        msg.pose.orientation.x = pose[3]
        msg.pose.orientation.y = pose[4]
        msg.pose.orientation.z = pose[5]
        msg.pose.orientation.w = pose[6]

        
        self._move_to_pose_publisher.publish(msg)

    def update_pose(self, pose:np.ndarray, update:np.ndarray):
        """
        Turn the array into pose and publish it
        Args:
            pose (np.ndarray): (7,) Pose as [x, y, z, qx, qy, qz, qq] 
                in meters and quaternions
            update (str): Update transformation as [dx, dy, dz] in meters

        TODO: Handle angles
        """
        pose[0] += update[0]
        pose[1] += update[1] 
        pose[2] += update[2]

        return pose

    def joy_callback(self, msg:Joy):
        """
        Subscriber callback for joy node.
        Args:
            msg (Joy): Message from joy node
        """

        # L(eft), R(ight), U(p), D(own)
        L_LR_AXIS = msg.axes[0]
        L_UD_AXIS = msg.axes[1]
        R_LR_AXIS = msg.axes[2]
        R_UD_AXIS = msg.axes[3]

        L_BUMPER = msg.buttons[9]
        R_BUMPER = msg.buttons[10]

        A_BUTTON = msg.buttons[1]
        B_BUTTON = msg.buttons[0]
        X_BUTTON = msg.buttons[3]
        Y_BUTTON = msg.buttons[2]

        # Swap arms 
        if Y_BUTTON:
            self.arm = 'right'
        elif A_BUTTON:
            self.arm = 'left'
    
        # Grasp handling
        grasp_msg = String()
        grasp_msg.data = self.arm
        if L_BUMPER:
            self._envelop_grasp_publisher.publish(grasp_msg)
        elif R_BUMPER:
            self._release_publisher.publish(grasp_msg)

        # Movement handling
        SCALAR = 0.008
        if L_UD_AXIS or L_LR_AXIS or R_UD_AXIS:
            # L_UD_AXIS flipped for easier viewing
            update = np.array([L_LR_AXIS, -L_UD_AXIS,R_UD_AXIS]) * SCALAR
            if self.arm == 'left':
                print("LEFT")
                self.cur_left_pose = self.update_pose(self.cur_left_pose, update)
                self.publish_pose(self.cur_left_pose, self.arm) 
            elif self.arm == 'right':
                print("RIGHT")
                self.cur_right_pose = self.update_pose(self.cur_right_pose, update)
                self.publish_pose(self.cur_right_pose, self.arm) 




def main(args=None):
    rclpy.init(args=args)
    node = JoyHandler()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()