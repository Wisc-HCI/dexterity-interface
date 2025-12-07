from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.parameter_descriptions import ParameterValue



def generate_launch_description():

    pkg_path = get_package_share_directory('robot_description')

    # --- Declare launch arguments ---
    declare_urdf_arg = DeclareLaunchArgument(
        'urdf_path',
        default_value=pkg_path + '/urdf/bimanual_arms.urdf',
        description='Path to the URDF or Xacro file to load into robot_state_publisher'
    )

    declare_rviz_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=pkg_path + '/config/bimanual_arm.rviz',
        description='Path to the RViz configuration file'
    )

    # Load launch argument values
    urdf_path = LaunchConfiguration('urdf_path')
    rviz_config = LaunchConfiguration('rviz_config')

    # --- Nodes ---
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            "robot_description": ParameterValue(
                Command(['xacro ', urdf_path]),
                value_type=str
            )
        }]
    )


    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    return LaunchDescription([
        declare_urdf_arg,
        declare_rviz_arg,
        robot_state_pub,
        rviz_node
    ])
