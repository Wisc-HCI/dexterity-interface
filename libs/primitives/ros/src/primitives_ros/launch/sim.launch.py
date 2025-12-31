from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable

def generate_launch_description():
    """
    Launch nodes needed to control them sim with primitive action servers.
    """

    return LaunchDescription([

        # Env variable for streaming to interface (set to 1 for GUI popup)
        SetEnvironmentVariable(
            name='LIVESTREAM',
            value='2'
        ),

        # Should be launched in container for correct file path
        Node(
            package='robot_motion_interface_ros',
            executable='interface',
            name='robot_motion_interface_sim',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'interface_type': 'isaacsim_object'},
                {'config_path': '/workspace/libs/robot_motion_interface/config/isaacsim_config.yaml'}
            ]
        ),

        Node(
            package='primitives_ros',
            executable='primitive_action_handler',
            name='primitives_sim',
            output='screen',
            emulate_tty=True,
        ),


    ])