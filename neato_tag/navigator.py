import rclpy
from lidar_detector import LidarDetector
from camera_detector import CameraDetector
from rclpy.context import Context
from rclpy.node import Node
from rclpy.parameter import Parameter
import numpy as np
from geometry_msgs.msg import Twist, Vector3

class Navigator(Node):

    def __init__(self, image_topic, lidar_topic):
        super().__init__('navigator')
        
        self.sub = self.create_subscription(Twist, 'cmd_vel')
        self.camera_subscriber=self.create_subscription(CameraDetector, image_topic, 10)
        self.camera_subscriber=self.create_subscription(LidarDetector, lidar_topic, 10)