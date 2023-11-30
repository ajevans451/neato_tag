
import rclpy
from rclpy.context import Context
from rclpy.node import Node
from rclpy.parameter import Parameter
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, Vector3
from cv_bridge import CvBridge

SHOWVISUALS=True

class CameraDetector(Node):
    def __init__(self, image_topic):
        super().__init__('camera_detector')

        self.cv_image = None                        # the latest image from the camera
        self.bridge=CvBridge()                  # used to convert ROS messages to OpenCV
        self.camera_subscriber=self.create_subscription(Image, image_topic, self.find_pink_neatos, 10)
        

    def find_pink_neatos(self,msg):
        self.cv_image = self.bridge.imgmsg_to_cv2(msg)
        image = cv2.cvtColor(self.cv_image, cv2.COLOR_RGB2BGR)
        # Convert RGB to HSV
        image_hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
        print(image_hsv)
        # define range of blue color in HSV
        lower_pink = np.array([322,7,50])
        upper_pink = np.array([336,95,95])
        # Threshold the HSV image to get only blue colors
        mask = cv2.inRange(image_hsv, lower_pink, upper_pink)

        if SHOWVISUALS==True: 
            # Bitwise-AND mask and original image
            res = cv2.bitwise_and(image,image, mask= mask)
            cv2.imshow('frame',image)
            cv2.imshow('mask',mask)
            cv2.imshow('res',res)
            k=cv2.waitKey(0)
            if k==27:
                cv2.destroyAllWindows
        
        
        


def main(args=None):
    rclpy.init()
    n = CameraDetector("camera/image_raw")
    rclpy.spin(n)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

if __name__ == '__main__':
    node = CameraDetector("/camera/image_raw")
    node.run()