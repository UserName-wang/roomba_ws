import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from rclpy.qos import QoSProfile

class RoombaController(Node):
    def __init__(self):
        super().__init__('roomba_controller')
        
        # 创建QoS配置
        qos_profile = QoSProfile(depth=10)
        
        # 创建订阅者，订阅/cmd_vel话题
        self.cmd_vel_subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            qos_profile)
        
        # 创建发布者，发布轮子速度命令
        self.wheel_cmd_publisher = self.create_publisher(
            Float64MultiArray,
            '/diff_drive_controller/commands',
            qos_profile)
        
        # 机器人参数
        self.wheel_radius = 0.03  # 轮子半径 (m)
        self.wheel_separation = 0.235  # 轮子间距 (m)
        
        self.get_logger().info('Roomba Controller node has been started')
        self.get_logger().info('Subscribing to /cmd_vel and publishing to /diff_drive_controller/commands')

    def cmd_vel_callback(self, msg):
        # 获取线速度和角速度
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        
        # 计算左右轮速度
        # 根据差速驱动运动学模型:
        # 左轮速度 = (线速度 - 角速度 * 轮距/2) / 轮子半径
        # 右轮速度 = (线速度 + 角速度 * 轮距/2) / 轮子半径
        left_wheel_speed = (linear_x - angular_z * self.wheel_separation / 2.0) / self.wheel_radius
        right_wheel_speed = (linear_x + angular_z * self.wheel_separation / 2.0) / self.wheel_radius
        
        # 创建消息并发布
        wheel_cmd_msg = Float64MultiArray()
        wheel_cmd_msg.data = [left_wheel_speed, right_wheel_speed]
        
        self.wheel_cmd_publisher.publish(wheel_cmd_msg)
        
        # 记录日志
        self.get_logger().debug(f'Cmd_vel: linear={linear_x:.2f}, angular={angular_z:.2f}')
        self.get_logger().debug(f'Wheel speeds: left={left_wheel_speed:.2f}, right={right_wheel_speed:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = RoombaController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()