import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class ControllerTester(Node):
    def __init__(self):
        super().__init__('controller_tester')
        
        # 创建发布者
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # 定时器
        self.timer = self.create_timer(2.0, self.timer_callback)
        self.counter = 0
        
        self.get_logger().info('Controller Tester node has been started')

    def timer_callback(self):
        msg = Twist()
        
        if self.counter % 4 == 0:
            # 前进
            msg.linear.x = 0.2
            msg.angular.z = 0.0
            self.get_logger().info('Moving forward')
        elif self.counter % 4 == 1:
            # 停止
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.get_logger().info('Stopping')
        elif self.counter % 4 == 2:
            # 后退
            msg.linear.x = -0.2
            msg.angular.z = 0.0
            self.get_logger().info('Moving backward')
        else:
            # 旋转
            msg.linear.x = 0.0
            msg.angular.z = 0.5
            self.get_logger().info('Turning')
            
        self.publisher_.publish(msg)
        self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    node = ControllerTester()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()