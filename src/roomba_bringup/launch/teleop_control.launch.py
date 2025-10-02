import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Launch arguments
    use_sim_arg = DeclareLaunchArgument(
        'use_sim',
        default_value='false',
        description='Use simulation (Gazebo)'
    )

    # Get package directories
    roomba_bringup_dir = get_package_share_directory('roomba_bringup')

    # Include ros2_control launch file with joystick enabled
    ros2_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(roomba_bringup_dir, 'launch', 'ros2_control.launch.py')
        ),
        launch_arguments={
            'use_joystick': 'true',
            'use_sim': LaunchConfiguration('use_sim')
        }.items()
    )

    return LaunchDescription([
        use_sim_arg,
        ros2_control_launch,
    ])
