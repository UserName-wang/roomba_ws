from setuptools import setup

package_name = 'roomba_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', 
            ['launch/display.launch.py',
             'launch/display_with_tf.launch.py',
             'launch/gazebo.launch.py',
             'launch/roomba.launch.py',
             'launch/ros2_control.launch.py',
             'launch/simulation.launch.py',
             'launch/teleop_control.launch.py',
             'launch/tf_demo.launch.py']),
        ('share/' + package_name + '/config', 
            ['config/roomba.rviz',
             'config/roomba_controllers.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='panda',
    maintainer_email='panda@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'roomba_node = roomba_bringup.roomba_node:main',
                'robot_simulator = roomba_bringup.robot_simulator:main',
                'tf_publisher = roomba_bringup.tf_publisher:main',
                'test_controller = roomba_bringup.test_controller:main',
                'roomba_controller = roomba_bringup.roomba_controller:main',
        ],
    },
)