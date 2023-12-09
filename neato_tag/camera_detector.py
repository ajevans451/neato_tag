
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image
from nav2_msgs.msg import ParticleCloud, Particle
from geometry_msgs.msg import Pose
from cv_bridge import CvBridge

"""
Color ranges for RGB masking
BLUE:
red- 0-26
green- 52-104
blue- 80-

YELLOW:
red- 123-194
green- 137-203
blue- 0-69

GREEN:
red- 28-98
green- 103-255
blue- 50-92
"""

SHOWVISUALS = True
LOWER_PINK = np.array([116,0,69])
UPPER_PINK = np.array([255,73,150])
LOWER_BLUE = np.array([0,52,80])
UPPER_BLUE = np.array([26,104,255])
LOWER_YELLOW = np.array([123,137,0])
UPPER_YELLOW = np.array([194,203,69])
LOWER_GREEN = np.array([28,103,50])
UPPER_GREEN = np.array([98,255,92])
CALIBRATION_MATRIX = np.array([[584.9932,0,0], [0, 584.6622,0], [377.3949,225.2839,1]])
FOCAL_Y = CALIBRATION_MATRIX[1, 1]
NOTE_HEIGHT = 0.0762


class CameraDetector(Node):
    def __init__(self):
        super().__init__('camera_detector')

        self.cv_image = None                        # the latest image from the camera
        self.bridge = CvBridge()                  # used to convert ROS messages to OpenCV
        self.camera_subscriber = self.create_subscription(Image, 'camera/image_raw', self.find_pink_neatos, 10)
        self.camera_subscriber = self.create_subscription(Image, 'camera/image_raw', self.find_blue_neatos, 10)
        self.camera_subscriber = self.create_subscription(Image, 'camera/image_raw', self.find_yellow_neatos, 10)
        self.camera_subscriber = self.create_subscription(Image, 'camera/image_raw', self.find_green_neatos, 10)
        self.publisher = self.create_publisher(ParticleCloud, 'neatos_in_camera', 10)
        cv2.namedWindow('frame')
        cv2.namedWindow('mask')


    def find_pink_neatos(self, msg):
        self.cv_image = self.bridge.imgmsg_to_cv2(msg)
        image = cv2.cvtColor(self.cv_image, cv2.COLOR_RGB2BGR)
        mask = cv2.inRange(self.cv_image, LOWER_PINK, UPPER_PINK)

        if SHOWVISUALS:
            cv2.imshow('frame',image)
            cv2.imshow('pink mask',mask)
            k=cv2.waitKey(10)
            if k==27:
                cv2.destroyAllWindows()
        
        blobs, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        big_blob = max(blobs, key=lambda b: b.shape[0]).reshape((-1, 2))
        blob_center_pix = np.mean(big_blob, axis=0)+0.5 #.5 added to account for pixel center
        blob_max_y = np.max(big_blob[:, 1])
        blob_min_y = np.min(big_blob[:, 1])
        blob_height_pix = blob_max_y - blob_min_y
        # print(blob_center_pix, blob_height_pix)

        z_distance = FOCAL_Y*NOTE_HEIGHT/blob_height_pix
        center_homogenous = np.array([blob_center_pix[0], blob_center_pix[1], 1])
        dir_3d = np.linalg.inv(CALIBRATION_MATRIX) @ center_homogenous
        point_in_3D = z_distance * dir_3d / dir_3d[2]
        
        pose = Pose()
        # print(point_in_3D)
        pose.position.x = point_in_3D[0]
        pose.position.y = point_in_3D[2]
        particle = Particle(pose=pose, weight=1.0)
        cloud = ParticleCloud()
        cloud.header = msg.header
        cloud.particles = [particle]
        self.publisher.publish(cloud)

    def find_blue_neatos(self, msg):
        self.cv_image = self.bridge.imgmsg_to_cv2(msg)
        image = cv2.cvtColor(self.cv_image, cv2.COLOR_RGB2BGR)
        mask = cv2.inRange(self.cv_image, LOWER_BLUE, UPPER_BLUE)

        if SHOWVISUALS:
            cv2.imshow('blue mask',mask)
            k=cv2.waitKey(10)
            if k==27:
                cv2.destroyAllWindows()
        
        blobs, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        big_blob = max(blobs, key=lambda b: b.shape[0]).reshape((-1, 2))
        blob_center_pix = np.mean(big_blob, axis=0)+0.5 #.5 added to account for pixel center
        blob_max_y = np.max(big_blob[:, 1])
        blob_min_y = np.min(big_blob[:, 1])
        blob_height_pix = blob_max_y - blob_min_y
        # print(blob_center_pix, blob_height_pix)

        z_distance = FOCAL_Y*NOTE_HEIGHT/blob_height_pix
        center_homogenous = np.array([blob_center_pix[0], blob_center_pix[1], 1])
        dir_3d = np.linalg.inv(CALIBRATION_MATRIX) @ center_homogenous
        point_in_3D = z_distance * dir_3d / dir_3d[2]
        
        pose = Pose()
        # print(point_in_3D)
        pose.position.x = point_in_3D[0]
        pose.position.y = point_in_3D[2]
        particle = Particle(pose=pose, weight=1.0)
        cloud = ParticleCloud()
        cloud.header = msg.header
        cloud.particles = [particle]
        self.publisher.publish(cloud)

    def find_yellow_neatos(self, msg):
        self.cv_image = self.bridge.imgmsg_to_cv2(msg)
        image = cv2.cvtColor(self.cv_image, cv2.COLOR_RGB2BGR)
        mask = cv2.inRange(self.cv_image, LOWER_YELLOW, UPPER_YELLOW)

        if SHOWVISUALS:
            cv2.imshow('yellow mask',mask)
            k=cv2.waitKey(10)
            if k==27:
                cv2.destroyAllWindows()
        
        blobs, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        big_blob = max(blobs, key=lambda b: b.shape[0]).reshape((-1, 2))
        blob_center_pix = np.mean(big_blob, axis=0)+0.5 #.5 added to account for pixel center
        blob_max_y = np.max(big_blob[:, 1])
        blob_min_y = np.min(big_blob[:, 1])
        blob_height_pix = blob_max_y - blob_min_y
        # print(blob_center_pix, blob_height_pix)

        z_distance = FOCAL_Y*NOTE_HEIGHT/blob_height_pix
        center_homogenous = np.array([blob_center_pix[0], blob_center_pix[1], 1])
        dir_3d = np.linalg.inv(CALIBRATION_MATRIX) @ center_homogenous
        point_in_3D = z_distance * dir_3d / dir_3d[2]
        
        pose = Pose()
        # print(point_in_3D)
        pose.position.x = point_in_3D[0]
        pose.position.y = point_in_3D[2]
        particle = Particle(pose=pose, weight=1.0)
        cloud = ParticleCloud()
        cloud.header = msg.header
        cloud.particles = [particle]
        self.publisher.publish(cloud)
    
    def find_green_neatos(self, msg):
        self.cv_image = self.bridge.imgmsg_to_cv2(msg)
        image = cv2.cvtColor(self.cv_image, cv2.COLOR_RGB2BGR)
        mask = cv2.inRange(self.cv_image, LOWER_GREEN, UPPER_GREEN)

        if SHOWVISUALS:
            cv2.imshow('green mask',mask)
            k=cv2.waitKey(10)
            if k==27:
                cv2.destroyAllWindows()
        
        blobs, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        big_blob = max(blobs, key=lambda b: b.shape[0]).reshape((-1, 2))
        blob_center_pix = np.mean(big_blob, axis=0)+0.5 #.5 added to account for pixel center
        blob_max_y = np.max(big_blob[:, 1])
        blob_min_y = np.min(big_blob[:, 1])
        blob_height_pix = blob_max_y - blob_min_y
        # print(blob_center_pix, blob_height_pix)

        z_distance = FOCAL_Y*NOTE_HEIGHT/blob_height_pix
        center_homogenous = np.array([blob_center_pix[0], blob_center_pix[1], 1])
        dir_3d = np.linalg.inv(CALIBRATION_MATRIX) @ center_homogenous
        point_in_3D = z_distance * dir_3d / dir_3d[2]
        
        pose = Pose()
        # print(point_in_3D)
        pose.position.x = point_in_3D[0]
        pose.position.y = point_in_3D[2]
        particle = Particle(pose=pose, weight=1.0)
        cloud = ParticleCloud()
        cloud.header = msg.header
        cloud.particles = [particle]
        self.publisher.publish(cloud)

def main(args=None):
    rclpy.init()
    n = CameraDetector()
    rclpy.spin(n)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
