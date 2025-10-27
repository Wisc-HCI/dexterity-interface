from robot_motion_interface.interface import Interface
from robot_motion_interface.panda.panda_interface import PandaInterface
from robot_motion_interface.tesollo.tesollo_interface import TesolloInterface
# from robot_motion_interface.isaacsim.isaacsim_interface import IsaacsimInterface
from robot_motion_interface.bimanual_interface import BimanualInterface

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.parameter import Parameter
from rclpy.node import Node
from sensor_msgs.msg import JointState
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
        self.declare_parameter('set_joint_state_topic', 'set_joint_state')
        self.declare_parameter('home_topic', 'home')

        interface_type = self.get_parameter('interface_type').value
        config_path = self.get_parameter('config_path').value
        publish_period = self.get_parameter('publish_period').value
        set_joint_state_topic = self.get_parameter('set_joint_state_topic').value
        home_topic = self.get_parameter('home_topic').value
        
        #################### Interfaces ####################
        if interface_type == "panda":
            self._interface = PandaInterface.from_yaml(config_path)
        elif interface_type == "tesollo":
            self._interface = TesolloInterface.from_yaml(config_path)
        elif interface_type == "isaacsim":
            # TODO
            # self._interface = IsaacsimInterface.from_yaml(config_path)
            pass
        elif interface_type == "bimanual":
            self._interface = BimanualInterface.from_yaml(config_path)
            BimanualInterface
        else:
            error_msg = "Invalid interface provided. Options: 'panda', 'tesollo', 'isaacsim', 'bimanual'"
            self.get_logger().error(error_msg)
            raise ValueError(error_msg)


        #################### Subscribers ####################
        self.create_subscription(JointState,set_joint_state_topic, self.set_joint_state_callback, 10)
        self.create_subscription(Empty, home_topic, self.home_callback, 10)

        #################### Publishers ####################
        self.create_timer(publish_period, self.joint_state_callback)
        
        self._interface.home()
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

        Return:
           (JointState): Joint position (rad) at msg.position and
                joint names at msg.name.
        """
        msg = JointState()
        msg.position = self._interface.joint_state()
        msg.name = self._interface.joint_names()
        return msg
    

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


    try:
        rclpy.spin(interface_node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        interface_node.shutdown()
        interface_node.destroy_node()
        rclpy.try_shutdown()



if __name__ == '__main__':
    main()