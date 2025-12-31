from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch nodes needed to control primitives with a gamepad controller.
    """
    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node'
        ),
        Node(
            package='primitives_ros',
            executable='joy_handler',
            name='joy_handler'
        ),
        Node(
            package='primitives_ros',
            executable='primitive_handler',
            name='primitive_handler'
        ),
    ])