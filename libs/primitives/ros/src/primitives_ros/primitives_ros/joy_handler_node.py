from primitives_ros.utils.transformation_utils import pose_to_transformation, transformation_to_pose, euler_to_quaternion
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
        self.declare_parameter('home_topic', '/home')  # Not primitive

        # Node topics
        joy_topic = self.get_parameter('joy_topic').value

        # Primitive topics
        primitive_envelop_grasp_topic = self.get_parameter('primitive_envelop_grasp_topic').value
        primitive_release_topic = self.get_parameter('primitive_release_topic').value
        primitive_move_to_pose_topic = self.get_parameter('primitive_move_to_pose_topic').value
        home_topic = self.get_parameter('home_topic').value

        #################### Class variables ####################
        self.arm = 'left'    
        self.left_home_pose = np.array([-0.259, -0.092,  0.426, 0.9234, -0.3839, 0.0, 0.0])
        self.T_cur_left_pose = pose_to_transformation(self.left_home_pose)

        self.right_home_pose = np.array([0.259, -0.092,  0.426, 0.3839,  0.9234, 0.0, 0.0])
        self.T_cur_right_pose = pose_to_transformation(self.right_home_pose)

    
        #################### Subscribers ####################
        self.create_subscription(Joy, joy_topic, self.joy_callback,10)

        #################### Publishers ####################
        self._envelop_grasp_publisher = self.create_publisher(String, primitive_envelop_grasp_topic, 10)
        self._release_publisher = self.create_publisher(String, primitive_release_topic, 10)
        self._move_to_pose_publisher = self.create_publisher(PoseStamped, primitive_move_to_pose_topic, 10)
        self._home_publisher = self.create_publisher(Empty, home_topic, 10)
        

         
    def publish_pose(self, pose:np.ndarray, arm: str):
        """
        Turn the array into pose and publish it
        Args:
            pose (np.ndarray): (7,) Pose as [x, y, z, qx, qy, qz, qw] 
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

    def update_pose(self, T_pose:np.ndarray, euler_update:np.ndarray):
        """
        Update T_pose based on the euler update array. Translation is relative
        to world, rotation is relative to end-effector.
        Args:
            T_pose (np.ndarray): (4,4) Pose as transformation matrix
            euler_update (np.ndarray): Update as [dx, dy, dz, droll, dpitch, dyaw] 
                in meters and degrees
        Returns:
            (np.ndarray): (4,4) Updates pose as transformation matrix
        """
        translation = euler_update[:3]
        quat = euler_to_quaternion(euler_update[3:])
        T_update = pose_to_transformation(np.concatenate((translation, quat))) 

        T_translation = np.eye(4)
        T_translation[:3, 3] = T_update[:3, 3]
        T_rotation = np.eye(4)
        T_rotation[:3, :3] = T_update[:3, :3]

        # Apply translation relative to world frame and rotation relative to 
        # local frame (end-effector)
        T_pose = T_translation @ T_pose @ T_rotation 

        return T_pose

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

        L_ARROW = msg.buttons[13]
        R_ARROW = msg.buttons[14]
        U_ARROW = msg.buttons[11]
        D_ARROW = msg.buttons[12]

        A_BUTTON = msg.buttons[1]
        B_BUTTON = msg.buttons[0]
        X_BUTTON = msg.buttons[3]
        Y_BUTTON = msg.buttons[2]

        START_BUTTON = msg.buttons[6]

        # MAPPING
        LEFT_ARM = Y_BUTTON
        RIGHT_ARM = A_BUTTON
        GRASP = L_BUMPER
        RELEASE = R_BUMPER
        DX = L_LR_AXIS
        DY = -L_UD_AXIS # Flipped bc current teleop is from opposite view as coordinate system
        DZ = R_UD_AXIS  
        DROLL = R_ARROW -  L_ARROW
        DPITCH = U_ARROW - D_ARROW
        DYAW = X_BUTTON - B_BUTTON
        HOME = START_BUTTON


        # Swap arms 
        if LEFT_ARM:
            self.arm = 'right'
        elif RIGHT_ARM:
            self.arm = 'left'

        # Home
        if HOME:
            self.T_cur_left_pose = pose_to_transformation(self.left_home_pose)
            self.T_cur_right_pose = pose_to_transformation(self.right_home_pose)
            self._home_publisher.publish(Empty())

            return
    
        # Grasp handling
        grasp_msg = String()
        grasp_msg.data = self.arm
        if GRASP:
            self.get_logger().warn(f"ARM: {self.arm}")
            self._envelop_grasp_publisher.publish(grasp_msg)
        elif RELEASE:
            self._release_publisher.publish(grasp_msg)

        # Movement handling
        TRANSLATE_SCALAR = 0.008
        ROTATE_SCALAR = 1
        if DX or DY or DZ or DROLL or DPITCH or DYAW:
            
            scaled_translation = np.array([DX, DY ,DZ]) * TRANSLATE_SCALAR
            scaled_rotation = np.array([DROLL, DPITCH, DYAW]) * ROTATE_SCALAR
            update = np.concatenate((scaled_translation, scaled_rotation))

            if self.arm == 'left':
                self.T_cur_left_pose = self.update_pose(self.T_cur_left_pose, update)
                left_pose = transformation_to_pose(self.T_cur_left_pose)
                self.publish_pose(left_pose, self.arm) 
            elif self.arm == 'right':
                self.T_cur_right_pose = self.update_pose(self.T_cur_right_pose, update)
                right_pose = transformation_to_pose(self.T_cur_right_pose)
                self.publish_pose(right_pose, self.arm) 




def main(args=None):
    rclpy.init(args=args)
    node = JoyHandler()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()