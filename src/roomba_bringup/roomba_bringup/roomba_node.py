import rclpy
from rclpy.node import Node

class RoombaNode(Node):
    def __init__(self):
        super().__init__('roomba_node')
        self.get_logger().info('Roomba node has been started')

def main(args=None):
    rclpy.init(args=args)
    node = RoombaNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()