import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class MultiConnect(Node):
    """
    A mux Node to control multiple Neatos using a single teleop node
    """
    def __init__(self, num_bots: int):
        """
        Sets up the multi_connect node to connect to some number of neatos given by num_bots

        Assumes that each neato is listening to a namespace given by robot{idx}, for idx in range(num_bots)
        For instance, the first neato would be listening to robot0/cmd_vel

        Args:
            num_bots: an int, the number of neatos to control
        """
        super().__init__('multi_connect')
        self.sub = self.create_subscription(Twist, 'cmd_vel', self.split_to_robots, 10)
        self.pubs = [self.create_publisher(Twist, f'robot{i}/cmd_vel', 10) for i in range(num_bots)]
    
    def split_to_robots(self, cmd: Twist):
        """
        Callback function for cmd_vel; sends the command to each neato

        Args:
            cmd: a Twist message, the message received by cmd_vel to be sent to each individual neato's cmd_vel
        """
        for pub in self.pubs:
            pub.publish(cmd)


def main():
    rclpy.init()
    n = MultiConnect(3)
    rclpy.spin(n)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
