import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class MultiConnect(Node):
    def __init__(self, num_bots: int):
        super().__init__('multi_connect')
        self.sub = self.create_subscription(Twist, 'cmd_vel', self.split_to_robots, 10)
        self.pubs = [self.create_publisher(Twist, f'robot{i}/cmd_vel', 10) for i in range(num_bots)]
    
    def split_to_robots(self, cmd: Twist):
        for pub in self.pubs:
            pub.publish(cmd)


def main():
    rclpy.init()
    n = MultiConnect(3)
    rclpy.spin(n)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
