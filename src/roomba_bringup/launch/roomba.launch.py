import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Launch arguments
    use_joystick_arg = DeclareLaunchArgument(
        'use_joystick',
        default_value='false',
        description='Use joystick for teleoperation'
    )
    
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Use RViz for visualization'
    )

    # Get package directories
    roomba_bringup_dir = get_package_share_directory('roomba_bringup')

    # Include ros2_control launch file
    ros2_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(roomba_bringup_dir, 'launch', 'ros2_control.launch.py')
        ),
        launch_arguments={
            'use_joystick': LaunchConfiguration('use_joystick'),
            'use_sim': 'false'
        }.items()
    )

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(roomba_bringup_dir, 'config', 'roomba.rviz')],
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )

    return LaunchDescription([
        use_joystick_arg,
        use_rviz_arg,
        ros2_control_launch,
        rviz_node,
    ])
