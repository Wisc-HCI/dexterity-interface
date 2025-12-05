import numpy as np
import rclpy
import threading
from rclpy.executors import ExternalShutdownException
from rclpy.parameter import Parameter
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Empty



class InterfaceNode(Node):

    def __init__(self):
        """
        Creates ROS wrapper for robot motion interfaces.
        Args:
            interface_type (str):Type of robot interface to use. Options:
                "panda", "tesollo", "isaacsim".
            config_path (str): Path to the yaml configuration file for the 
                interface (see python interface files in robot_motion_interface)
                for details of what is needed in each config file/
            publish_period (float): Time period between published state updates. 
                Defaults: 0.1 s (10 Hz)
            set_joint_state_topic (str): Name of the topic used to send 
                joint state commands. Default: "set_joint_state"
            home_topic (str): Name of the topic used to send 
                home the robot. Default: "home"
        """
        super().__init__('interface_node')
        
        #################### Parameters ####################
        # Interface specific
        self.declare_parameter('interface_type', Parameter.Type.STRING)
        self.declare_parameter('config_path', Parameter.Type.STRING)
        # Node customization
        self.declare_parameter('publish_period', 0.1)  # 10 hz default
        self.declare_parameter('joint_state_topic', '/joint_state')
        self.declare_parameter('set_joint_state_topic', '/set_joint_state')
        self.declare_parameter('set_cartesian_pose_topic', '/set_cartesian_pose')
        self.declare_parameter('home_topic', '/home')

        interface_type = self.get_parameter('interface_type').value
        config_path = self.get_parameter('config_path').value
        publish_period = self.get_parameter('publish_period').value
        joint_state_topic = self.get_parameter('joint_state_topic').value
        set_joint_state_topic = self.get_parameter('set_joint_state_topic').value
        set_cartesian_pose_topic = self.get_parameter('set_cartesian_pose_topic').value
        home_topic = self.get_parameter('home_topic').value
        
        #################### Interfaces ####################
        # Only import at runtime to avoid dependency errors
        if interface_type == "panda":
            from robot_motion_interface.panda.panda_interface import PandaInterface
            self._interface = PandaInterface.from_yaml(config_path)
        elif interface_type == "tesollo":
            from robot_motion_interface.tesollo.tesollo_interface import TesolloInterface
            self._interface = TesolloInterface.from_yaml(config_path)
        elif interface_type == "isaacsim":
            # Prevent ros args from trickling down and causing isaacsim errors
            import sys
            sys.argv = sys.argv[:1]

            from robot_motion_interface.isaacsim.isaacsim_interface import IsaacsimInterface
            self._interface = IsaacsimInterface.from_yaml(config_path)
            pass
        elif interface_type == "bimanual":
            from robot_motion_interface.bimanual_interface import BimanualInterface
            self._interface = BimanualInterface.from_yaml(config_path)

        else:
            error_msg = "Invalid interface provided. Options: 'panda', 'tesollo', 'isaacsim', 'bimanual'"
            self.get_logger().error(error_msg)
            raise ValueError(error_msg)


        #################### Subscribers ####################
        self.create_subscription(JointState,set_joint_state_topic, self.set_joint_state_callback, 10)
        self.create_subscription(PoseStamped, set_cartesian_pose_topic, self.set_cartesian_pose_callback, 10)
        self.create_subscription(Empty, home_topic, self.home_callback, 10)

        #################### Publishers ####################
        self.joint_state_publisher_ = self.create_publisher(JointState, joint_state_topic, 10)
        self.create_timer(publish_period, self.joint_state_callback)


        # self._interface.home()

        
    def start(self):
        """ 
        Starts blocking loop
        """
        self._interface.start_loop()

    def set_joint_state_callback(self, msg:JointState):
        """
        Subscriber callback function for receiving and applying joint 
        state commands(non-blocking).

        Args:
            msg (JointState): Requires joint position (rad) at msg.position and
                joint names at msg.name.
        """
        q = np.array(msg.position, dtype=float)
        joint_names = msg.name

        # Non-blocking since subscriber (instead of service)
        self._interface.set_joint_positions(q, joint_names, False)
        

    def joint_state_callback(self):
        """
        Publisher callback function for publishing joint state commands.
        Publishes joint position (rad) at msg.position, joint velocity (rad/s)
        at msg.velocity, and joint names at msg.name.
        """
        
        state = self._interface.joint_state()
        if state is None or state.size == 0:
            return
        
        names = self._interface.joint_names()
        n_joints = len(names)
        positions = state[:n_joints]
        velocities = state[:n_joints]

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.position = positions.tolist()
        msg.velocity = velocities.tolist()
        msg.name = names
       
        self.joint_state_publisher_.publish(msg)
    
    def set_cartesian_pose_callback(self, msg:PoseStamped):
        """
        Subscriber callback function for receiving and applying cartesian 
        commands(non-blocking).

        Args:
            msg (PoseStamped): Requires ee frame (link name) at msg.header.frame_id and
                pose at msg.pose.position.x/y/z (m) and msg.pose.orientation.x/y/z/w (quat). 
        """
        pos = msg.pose.position
        ori = msg.pose.orientation
        x_list = np.array([[pos.x, pos.y, pos.z, ori.x, ori.y, ori.z, ori.w]], dtype=float)
        frames = [msg.header.frame_id]
        

        # Non-blocking since subscriber (instead of service)
        self._interface.set_cartesian_pose(x_list, frames)
        
    def home_callback(self, msg: Empty):
        """
        Subscriber callback for homing the robot (non-blocking).
        Args:
            msg (Empty): Empty message just to trigger.
        """
        self._interface.home(False)


    # TODO: Cartesian pose

    def shutdown(self):
        """
        Shutdowns node properly
        """
        self._interface.stop_loop()


def main(args=None):
    rclpy.init(args=args)

    interface_node = InterfaceNode()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(interface_node)

    # Interfaces like Isaacsim must be in main thread which means
    # ROS node must be in its own thread
    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()

    try:
        interface_node.start()
        # Keep this alive for non-blocking loops (panda, tesollo)
        while(True):
            pass  # TODO: See if need to pause
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        interface_node.shutdown()
        interface_node.destroy_node()
        rclpy.try_shutdown()



if __name__ == '__main__':
    main()