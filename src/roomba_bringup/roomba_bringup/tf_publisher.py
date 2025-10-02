import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math

class TFPublisher(Node):
    def __init__(self):
        super().__init__('tf_publisher')
        
        # 创建TF广播器
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # 创建定时器，定期发布TF变换
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        # 初始化时间
        self.time_elapsed = 0.0
        
        self.get_logger().info('TF Publisher node has been started')

    def timer_callback(self):
        # 创建变换消息
        t = TransformStamped()
        
        # 设置变换的header
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        
        # 设置变换的平移部分
        # 让机器人沿圆形路径移动
        self.time_elapsed += 0.1
        radius = 1.0  # 圆形路径半径
        angular_velocity = 0.5  # 角速度
        
        t.transform.translation.x = radius * math.cos(angular_velocity * self.time_elapsed) - radius
        t.transform.translation.y = radius * math.sin(angular_velocity * self.time_elapsed)
        t.transform.translation.z = 0.0
        
        # 设置变换的旋转部分（使用四元数）
        # 机器人朝向运动方向
        yaw = angular_velocity * self.time_elapsed + math.pi/2
        
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(yaw / 2)
        t.transform.rotation.w = math.cos(yaw / 2)
        
        # 发布变换
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = TFPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()