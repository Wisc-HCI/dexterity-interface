from setuptools import find_packages, setup

package_name = 'robot_motion_interface_ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
    },
    entry_points={
        'console_scripts': [
            'interface = robot_motion_interface_ros.interface_node:main',
        ],
    },
)
