from setuptools import find_packages, setup
import os
from glob import glob

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
    maintainer='Mya Schroder',
    maintainer_email='mcschroder@wisc.edu',
    description='ROS2 Wrapper for robot_motion_interface',
    license='TODO: License declaration',
    extras_require={
    },
    entry_points={
        'console_scripts': [
            'interface = robot_motion_interface_ros.interface_node:main',
        ],
    },
)