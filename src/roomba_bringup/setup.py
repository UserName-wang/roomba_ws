from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'roomba_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), 
            glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'urdf'), 
            glob(os.path.join('urdf', '*.urdf*'))),
        (os.path.join('share', package_name, 'config'), 
            glob(os.path.join('config', '*.rviz')) +
            glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='panda',
    maintainer_email='your.email@example.com',
    description='Roomba robot bringup package',
    license='Apache License 2.0',
    build_type='ament_cmake',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'roomba_node = roomba_bringup.roomba_node:main',
            'tf_publisher = roomba_bringup.tf_publisher:main',
            'robot_simulator = roomba_bringup.robot_simulator:main',
            'roomba_controller = roomba_bringup.roomba_controller:main',
            'test_controller = roomba_bringup.test_controller:main',
        ],
    },
)