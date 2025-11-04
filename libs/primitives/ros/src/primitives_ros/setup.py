from setuptools import find_packages, setup

package_name = 'primitives_ros'

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
    maintainer_email='mcschroder@wisc.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
    },
    entry_points={
        'console_scripts': [
            'primitive_handler = primitives_ros.primitive_handler_node:main',
            'joy_handler = primitives_ros.joy_handler_node:main',
        ],
    },
)
