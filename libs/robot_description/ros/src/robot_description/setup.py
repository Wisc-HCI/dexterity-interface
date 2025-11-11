from setuptools import find_packages, setup
import os
from glob import glob
 

package_name = 'robot_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    # data_files=[
    #     ('share/ament_index/resource_index/packages',
    #         ['resource/' + package_name]),
    #     ('share/' + package_name, ['package.xml']),
    #     (os.path.join('share', package_name, 'launch'), glob('launch/*')),
    #     (os.path.join('share', package_name, 'config'), glob('config/*')),
    #     (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
    #     (os.path.join('share', package_name, 'urdf/panda'), glob('urdf/panda/*')),
    #     (os.path.join('share', package_name, 'urdf/panda/meshes/collision'), glob('urdf/panda/meshes/collision/*')),
    #     (os.path.join('share', package_name, 'urdf/panda/meshes/visual'), glob('urdf/panda/meshes/visual/*')), # TODO: Make this part cleaner as adding all folders currently manually
    # ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),

        (os.path.join('share', package_name, 'urdf', 'panda'), 
            glob('urdf/panda/*')),

        (os.path.join('share', package_name, 'urdf', 'panda', 'meshes', 'collision'),
            glob('urdf/panda/meshes/collision/*')),

        (os.path.join('share', package_name, 'urdf', 'panda', 'meshes', 'visual'),
            glob('urdf/panda/meshes/visual/*')),
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
        'console_scripts': [
        ],
    },
)
