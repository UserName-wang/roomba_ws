import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import Command

def generate_launch_description():
    # 获取包的share目录
    pkg_share = get_package_share_directory('roomba_description')
    
    # 定义xacro文件路径
    xacro_file_name = 'urdf/roomba.urdf.xacro'
    xacro_file = os.path.join(pkg_share, xacro_file_name)
    
    # 定义launch参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        
        # 发布robot_state_publisher节点
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time, 
                'robot_description': Command(['xacro ', xacro_file])
            }]),
        
        # 发布joint_state_publisher节点
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time
            }]),
        
        # 发布joint_state_publisher_gui节点（可选）
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time
            }]),
        
        # 启动RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(pkg_share, 'config', 'roomba.rviz')])
    ])