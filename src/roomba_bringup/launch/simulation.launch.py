import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
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
    roomba_description_dir = get_package_share_directory('roomba_description')

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        arguments=[os.path.join(roomba_description_dir, 'urdf', 'roomba.urdf')]
    )

    # Gazebo server
    gazebo_server = ExecuteProcess(
        cmd=['gzserver', '--verbose', '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    # Gazebo client
    gazebo_client = ExecuteProcess(
        cmd=['gzclient'],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )

    # Spawn entity
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'roomba', '-topic', 'robot_description'],
        output='screen'
    )

    # Include ros2_control launch file
    ros2_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(roomba_bringup_dir, 'launch', 'ros2_control.launch.py')
        ),
        launch_arguments={
            'use_joystick': LaunchConfiguration('use_joystick'),
            'use_sim': 'true'
        }.items()
    )

    return LaunchDescription([
        use_joystick_arg,
        use_rviz_arg,
        robot_state_publisher,
        gazebo_server,
        gazebo_client,
        spawn_entity,
        ros2_control_launch,
    ])