import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 获取包的share目录
    pkg_share = get_package_share_directory('roomba_bringup')
    
    # 定义URDF文件路径
    urdf_file_name = 'urdf/roomba.urdf'
    urdf = os.path.join(pkg_share, urdf_file_name)
    
    # 定义launch参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # 启动Gazebo服务器
    start_gazebo_server_cmd = ExecuteProcess(
        cmd=['gzserver', '--verbose', '-s', 'libgazebo_ros_factory.so'],
        output='screen')
    
    # 启动Gazebo客户端
    start_gazebo_client_cmd = ExecuteProcess(
        cmd=['gzclient'],
        output='screen')
    
    # 发布robot_state_publisher节点
    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
        }],
        arguments=[urdf])
    
    # 在Gazebo中生成机器人实体
    spawn_entity_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'roomba',
            '-file', urdf,
            '-x', '0',
            '-y', '0',
            '-z', '0.05'
        ],
        output='screen')
    
    # 创建launch描述
    ld = LaunchDescription()
    
    # 添加动作
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(spawn_entity_cmd)
    
    return ld