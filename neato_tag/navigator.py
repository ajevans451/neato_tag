import rclpy
from lidar_detector import LidarDetector
from camera_detector import CameraDetector
from rclpy.context import Context
from rclpy.node import Node
from rclpy.parameter import Parameter
import numpy as np
from nav2_msgs.msg import ParticleCloud, Particle
from geometry_msgs.msg import Twist, Vector3


turn_constant=5

class Navigator(Node):

    def __init__(self):
        super().__init__('navigator')

        self.camera_subscriber=self.create_subscription(ParticleCloud, 'neatos_in_camera', self.find_heading,10)
        self.lidar_subscriber=self.create_subscription(ParticleCloud,  'neatos_in_lidar', self.find_heading,10)
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.it_status=False

    def run_loop(self,msg):

        nearest_neato_distance=[]
        nearest_neato_angle=[]
        self.drive(nearest_neato_angle,nearest_neato_distance)
        
    def drive(self,angle, distance):
        """
        Takes in angle and distance to the nearest neato and commands Neato motors to head towards or away from that position
        """
        if self.it_status:
            forward_constant=.5
        else:
            forward_constant=-.5
        drive_msg = Twist()
        drive_msg.linear.x = np.clip(float(forward_constant*distance), -1.0, 1.0)    #Determines final linear drive message
        drive_msg.angular.z = np.clip(float(turn_constant*angle), -1.0, 1.0)         #Determines final angular drive message
        self.pub.publish(drive_msg) #Commands Neato motors
        return
        


        

def main(args=None):
    rclpy.init()
    n = Navigator()
    rclpy.spin(n)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
