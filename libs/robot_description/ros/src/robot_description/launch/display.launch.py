from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import yaml
from launch import LaunchDescription


# path_to_current_package = get_package_share_directory('robot_motion') + '/robot_motion/ik/'
# setting_file_path = path_to_current_package + 'config/' + 'bimanual_ik_settings.yaml' # need to change this probably as currently not working for settings.yaml
# print("Setting file path in display.launch.py:", setting_file_path)

def generate_launch_description():
    path_to_pkg = get_package_share_directory('robot_description')  

    # TODO: parameterize these
    urdf_path = path_to_pkg + '/urdf/bimanual_arms.urdf'
    urdf_file = open(urdf_path, 'r')
    urdf_string = urdf_file.read()

    rviz_cfg_path = path_to_pkg + '/config/bimanual_arm.rviz'


    return LaunchDescription([
        Node(
            package='rviz2',
            name='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d',rviz_cfg_path],
        ),
        # TODO: Allow parameter to update /joint_state topic to other topic
        Node(
            package='robot_state_publisher',
            name='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{ 
                        'robot_description': urdf_string}]
        )
    ])