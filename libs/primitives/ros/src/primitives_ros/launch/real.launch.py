from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch nodes needed to control the real bimanual system with primitive action servers.
    """

    return LaunchDescription([

        # Should be launched in container for correct file path
        Node(
            package='robot_motion_interface_ros',
            executable='interface',
            name='robot_motion_interface_sim',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'interface_type': 'bimanual'},
                {'config_path': '/workspace/libs/robot_motion_interface/config/bimanual_arm_config.yaml'},

                # Topic/action names
                {'joint_state_topic': '/joint_state/real'},
                {'set_joint_state_topic': '/set_joint_state/real'},
                {'set_cartesian_pose_topic': '/set_cartesian_pose/real'},
                {'home_topic': '/home/real'},
                {'set_joint_state_action': '/set_joint_positions/real'},
                {'set_cartesian_pose_action': '/set_cartesian_pose/real'},
                {'home_action': '/home/real'},
            ]
        ),

        Node(
            package='primitives_ros',
            executable='primitive_action_handler',
            name='primitives_sim',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'primitive_action': '/primitives/real'},
                {'home_action': '/home/real'},
                {'set_joint_state_action': '/set_joint_positions/real'},
                {'set_cartesian_pose_action': '/set_cartesian_pose/real'},
            ]
        ),
    ])