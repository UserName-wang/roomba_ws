import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    # Declare launch arguments
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyUSB0',
        description='Serial port for Roomba connection'
    )
    
    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate',
        default_value='115200',
        description='Baud rate for serial communication'
    )
    
    use_joystick_arg = DeclareLaunchArgument(
        'use_joystick',
        default_value='true',
        description='Use joystick for teleoperation'
    )
    
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace for the nodes'
    )

    # Roomba driver test node
    roomba_driver_test_node = Node(
        package='roomba_driver_test',
        executable='driver_test',
        name='roomba_driver_test',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('port'),
            'baud_rate': LaunchConfiguration('baud_rate')
        }],
        remappings=[
            ('cmd_vel', 'cmd_vel')
        ]
    )

    # Joystick teleoperation nodes
    teleop_group = GroupAction([
        PushRosNamespace('teleop'),
        
        # Joy node
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
        ),
        
        # Teleop twist joy node
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            output='screen',
            parameters=[{
                'axis_linear.x': 1,
                'axis_angular.yaw': 0,
                'scale_linear.x': 0.5,
                'scale_angular.yaw': 0.5
            }],
            remappings=[
                ('cmd_vel', '/cmd_vel')
            ]
        )
    ],
    condition=IfCondition(LaunchConfiguration('use_joystick'))
    )

    # Create launch description
    ld = LaunchDescription()
    
    # Add actions
    ld.add_action(port_arg)
    ld.add_action(baud_rate_arg)
    ld.add_action(use_joystick_arg)
    ld.add_action(namespace_arg)
    ld.add_action(roomba_driver_test_node)
    ld.add_action(teleop_group)
    
    return ld