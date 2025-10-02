import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 获取包的share目录
    pkg_share = get_package_share_directory('roomba_bringup')
    
    # 定义URDF文件路径
    urdf_file_name = 'urdf/roomba.urdf'
    urdf = os.path.join(pkg_share, urdf_file_name)
    
    # 读取URDF文件内容
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()
    
    # 定义launch参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        
        # 发布map到odom的静态变换
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher_map_odom',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
        ),
        
        # 发布robot_state_publisher节点
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time, 
                'robot_description': robot_desc
            }]),
        
        # 发布joint_state_publisher_gui节点（可选）
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'rate': 10,
            }]),
        
        # 启动机器人模拟器节点
        Node(
            package='roomba_bringup',
            executable='robot_simulator',
            name='robot_simulator',
            output='screen'
        ),
        
        # 启动RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(pkg_share, 'config', 'roomba.rviz')])
    ])