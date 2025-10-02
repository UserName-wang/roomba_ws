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
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Use RViz for visualization'
    )

    # Get package directories
    roomba_bringup_dir = get_package_share_directory('roomba_bringup')
    roomba_description_dir = get_package_share_directory('roomba_description')

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        arguments=[os.path.join(roomba_description_dir, 'urdf', 'roomba.urdf')]
    )

    # Include simulation launch file
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(roomba_bringup_dir, 'launch', 'simulation.launch.py')
        )
    )

    return LaunchDescription([
        use_rviz_arg,
        robot_state_publisher,
        simulation_launch,
    ])
