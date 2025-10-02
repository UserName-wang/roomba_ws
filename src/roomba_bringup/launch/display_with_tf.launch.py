import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
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

    # TF publisher
    tf_publisher = Node(
        package='roomba_bringup',
        executable='tf_publisher',
        name='tf_publisher',
        output='screen'
    )

    # Joint state publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
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
        use_rviz_arg,
        robot_state_publisher,
        tf_publisher,
        joint_state_publisher,
        rviz_node,
    ])
