from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import yaml
from launch import LaunchDescription

path_to_src = get_package_share_directory('robot_description')
print("Path to robot_motion package:", path_to_src)
# path_to_current_package = get_package_share_directory('robot_motion') + '/robot_motion/ik/'
# setting_file_path = path_to_current_package + 'config/' + 'bimanual_ik_settings.yaml' # need to change this probably as currently not working for settings.yaml
# print("Setting file path in display.launch.py:", setting_file_path)

def generate_launch_description():
    # Load the infomation
    # setting_file = open(setting_file_path, 'r')
    # settings = yaml.load(setting_file, Loader=yaml.FullLoader)

    urdf_path = path_to_src + '/urdf/bimanual_arms.urdf'
    print("URDF path:", urdf_path)
    urdf_file = open(urdf_path, 'r')
    urdf_string = urdf_file.read()
    print("URDF String:", urdf_string)

    # print("demo.launch: using setting file path", str(setting_file_path))

    return LaunchDescription([
        # Node(
        #     package='robot_motion',
        #     namespace='',
        #     executable='ranged_ik_node.py',
        #     name='ranged_ik_node',
        #     parameters=[
        #         {'use_visualization': True,
        #          'setting_file_path': setting_file_path}
        #     ]
        # ), 
        Node(
            package='rviz2',
            name='rviz2',
            executable='rviz2',
            output='screen',
            # arguments=['-d', path_to_current_package + "/config/relaxed_config.rviz"],
        ),
        Node(
            package='robot_state_publisher',
            name='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            arguments=[urdf_path],
            parameters=[{'publish_frequency': 50.0, 
                        'robot_description': urdf_string}]
        )
        # ,
        # Node(
        #     package='robot_motion',
        #     namespace='',
        #     executable='rviz_viewer.py',
        #     name='rviz_viewer',
        #     output='screen',
        #     parameters=[{'setting_file_path': setting_file_path}]
        # ),
    ])