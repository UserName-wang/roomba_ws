import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
import math

class RobotSimulator(Node):
    def __init__(self):
        super().__init__('robot_simulator')
        
        # 创建TF广播器
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # 订阅关节状态
        self.joint_state_subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10)
        
        # 初始化机器人状态
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # 机器人参数
        self.wheel_radius = 0.03  # 轮子半径
        self.wheel_separation = 0.235  # 轮子间距
        
        # 上次更新时间
        self.last_time = self.get_clock().now()
        
        # 关节位置
        self.left_wheel_pos = 0.0
        self.right_wheel_pos = 0.0
        self.last_left_wheel_pos = 0.0
        self.last_right_wheel_pos = 0.0
        
        self.get_logger().info('Robot Simulator node has been started')

    def joint_state_callback(self, msg):
        # 获取当前时间
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9  # 转换为秒
        
        if dt < 0.001:  # 避免时间间隔过小
            return
            
        # 找到左右轮的关节位置
        for i, name in enumerate(msg.name):
            if name == 'left_wheel_joint':
                self.left_wheel_pos = msg.position[i] if i < len(msg.position) else 0.0
            elif name == 'right_wheel_joint':
                self.right_wheel_pos = msg.position[i] if i < len(msg.position) else 0.0
        
        # 计算轮子角度变化
        delta_left = self.left_wheel_pos - self.last_left_wheel_pos
        delta_right = self.right_wheel_pos - self.last_right_wheel_pos
        
        # 计算机器人的线速度和角速度
        v_left = delta_left * self.wheel_radius / dt
        v_right = delta_right * self.wheel_radius / dt
        
        v = (v_left + v_right) / 2.0  # 线速度
        omega = (v_right - v_left) / self.wheel_separation  # 角速度
        
        # 更新机器人位置
        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt
        self.theta += omega * dt
        
        # 保持角度在[-pi, pi]范围内
        if self.theta > math.pi:
            self.theta -= 2 * math.pi
        elif self.theta < -math.pi:
            self.theta += 2 * math.pi
        
        # 发布TF变换
        self.publish_tf(current_time)
        
        # 更新上次状态
        self.last_left_wheel_pos = self.left_wheel_pos
        self.last_right_wheel_pos = self.right_wheel_pos
        self.last_time = current_time

    def publish_tf(self, time):
        # 创建变换消息
        t = TransformStamped()
        
        # 设置变换的header
        t.header.stamp = time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        
        # 设置变换的平移部分
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        
        # 设置变换的旋转部分（使用四元数）
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(self.theta / 2)
        t.transform.rotation.w = math.cos(self.theta / 2)
        
        # 发布变换
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = RobotSimulator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()