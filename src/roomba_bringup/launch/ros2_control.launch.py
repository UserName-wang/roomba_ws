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
    
    use_sim_arg = DeclareLaunchArgument(
        'use_sim',
        default_value='false',
        description='Use simulation (Gazebo)'
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

    # Controller manager
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[os.path.join(roomba_bringup_dir, 'config', 'roomba_controllers.yaml')],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_sim').__invert__())
    )

    # Joint state broadcaster
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )

    # Diff drive controller
    diff_drive_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller'],
        output='screen',
    )

    # Teleop twist joy node
    teleop_twist_joy = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_joystick'))
    )

    # Joy node
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_joystick'))
    )

    # Include simulation launch file if use_sim is true
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(roomba_bringup_dir, 'launch', 'simulation.launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('use_sim'))
    )

    return LaunchDescription([
        use_joystick_arg,
        use_sim_arg,
        robot_state_publisher,
        controller_manager,
        joint_state_broadcaster,
        diff_drive_controller,
        teleop_twist_joy,
        joy_node,
        simulation_launch,
    ])