from setuptools import find_packages, setup
import os
from glob import glob
 

package_name = 'robot_description'

# Get all URDF file paths, keeping same file structure
urdf_paths = [
    (os.path.join('share', package_name, root), [os.path.join(root, f) for f in files])
    for root, _, files in os.walk('urdf') if files
]

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),

    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        *urdf_paths
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jeffr',
    maintainer_email='jeffrey@theliuhome.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': ['frame_listener = robot_description.frame_listener_node:main'
        ],
    },
)
