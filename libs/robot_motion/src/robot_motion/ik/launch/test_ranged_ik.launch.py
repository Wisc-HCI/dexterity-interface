from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("robot_motion")
    # Adjust paths if your URDF or rviz config live elsewhere
    urdf_file = PathJoinSubstitution([pkg_share, "urdf", "bimanual_arms.urdf"]) 
    rviz_config = PathJoinSubstitution([pkg_share, "rviz", "ranged_ik.rviz"]) 

    use_rviz = DeclareLaunchArgument(
        "use_rviz", default_value="true", description="Whether to launch RViz"
    )

    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": False}],
        arguments=[urdf_file],
    )

    ranged_ik_node = Node(
        package="robot_motion",
        executable="ranged_ik_node.py",
        name="ranged_ik_test_node",
        output="screen",
        # if installed as a module/executable change package and executable accordingly
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        output="screen",
    )

    return LaunchDescription([
        use_rviz,
        robot_state_pub,
        ranged_ik_node,
        rviz,
    ])
