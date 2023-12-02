
import rclpy
from rclpy.context import Context
from rclpy.node import Node
from rclpy.parameter import Parameter
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, Vector3
from cv_bridge import CvBridge

SHOWVISUALS = False
LOWER_PINK = np.array([103,1,100])
UPPER_PINK = np.array([255,85,155])

class CameraDetector(Node):
    def __init__(self):
        super().__init__('camera_detector')

        self.cv_image = None                        # the latest image from the camera
        self.bridge = CvBridge()                  # used to convert ROS messages to OpenCV
        self.camera_subscriber = self.create_subscription(Image, 'camera/image_raw', self.find_pink_neatos, 10)

    def find_pink_neatos(self,msg):
        self.cv_image = self.bridge.imgmsg_to_cv2(msg)
        image = cv2.cvtColor(self.cv_image, cv2.COLOR_RGB2BGR)
        # Threshold the HSV image to get only blue colors
        mask = cv2.inRange(self.cv_image, LOWER_PINK, UPPER_PINK)

        if SHOWVISUALS: 
            # Bitwise-AND mask and original image
            res = cv2.bitwise_and(image,image, mask= mask)
            cv2.imshow('frame',image)
            cv2.imshow('mask',mask)
            cv2.imshow('res',res)
            k=cv2.waitKey(0)
            if k==27:
                cv2.destroyAllWindows
        
        blobs, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        big_blob = max(blobs, key=lambda b: b.shape[0]).reshape((-1, 2))
        blob_center_pix = np.mean(big_blob, axis=0)
        blob_max_y = np.max(big_blob[:, 1])
        blob_min_y = np.min(big_blob[:, 1])
        blob_height_pix = blob_max_y - blob_min_y
        print(blob_center_pix, blob_height_pix)


def main(args=None):
    rclpy.init()
    n = CameraDetector()
    rclpy.spin(n)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
