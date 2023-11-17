import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class MultiConnect(Node):

    def __init__(self):
        super().__init__('multi_connect')
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)


def main(args=None):
    rclpy.init()
    n = MultiConnect()
    rclpy.spin(n)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

if __name__ == '__main__':
    node = MultiConnect()
    node.run()

