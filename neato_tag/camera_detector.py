
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image
from nav2_msgs.msg import ParticleCloud, Particle
from geometry_msgs.msg import Pose
from cv_bridge import CvBridge
from neato_tag.game import CALIBRATION_MATRIX, FOCAL_Y, NOTE_HEIGHT, NEATO_TAG, COLOR_TO_MASK
from typing import Optional

SHOWVISUALS = False


class CameraDetector(Node):
    """
    A Node to find every Neato visible by the camera

    Performs a color mask on the camera image for each neato in the game, and, for each resulting neato,
    calculates its position in 3D space. This node will publish this information as a ParticleCloud topic
    with the name neatos_in_camera.
    """
    def __init__(self):
        """
        Sets up this camera detector node
        """
        super().__init__('camera_detector')
        self.bridge = CvBridge()                  # used to convert ROS messages to OpenCV
        self.camera_subscriber = self.create_subscription(Image, 'camera/image_raw', self.find_neatos, 10)
        self.publisher = self.create_publisher(ParticleCloud, 'neatos_in_camera', 10)
        if SHOWVISUALS:
            cv2.namedWindow('frame')
            cv2.namedWindow('mask')
    
    def find_neatos(self, msg: Image):
        """
        Find all neatos within a camera image, and publish their locations to neatos_in_camera

        Args:
            msg: an Image message holding the image snapshot from the camera
        """
        # Initialize resulting message
        cloud = ParticleCloud()
        cloud.header = msg.header

        # Parse the image and convert it to HSV for color masking
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        if SHOWVISUALS:
            cv2.imshow('frame', cv_image)
            k=cv2.waitKey(10)
            if k==27:
                cv2.destroyAllWindows()

        # For each player, find the neato with that mask
        for idx, color in enumerate(NEATO_TAG.player_colors):
            particle = self.find_neato_from_mask(hsv_image, *COLOR_TO_MASK[color], color.lower())
            if particle is not None:
                # Here, the weight represents the ID of the detected neato (for debugging purposes)
                particle.weight = float(idx)
                cloud.particles.append(particle)

        # Once all neatos are searched for, publish the cloud
        self.publisher.publish(cloud)

    def find_neato_from_mask(self, cv_image, lower_mask: np.ndarray, upper_mask: np.ndarray, color: str) -> Optional[Particle]:
        """
        Given an OpenCV image and a colormask, find the neato with that color and return its position as a Particle

        Args:
            cv_image: the OpenCV image from the camera
            lower_mask: a 3D numpy vector giving the minimum value for each of the three channels in cv_image
            upper_mask: a 3D numpy vector giving the maximum value for each of the three channels in cv_image
            color: a string, the name of the color (used to name the window when SHOWVISUALS is True)

        Returns:
            a Particle where the X and Y values give the coordinates of the neato found with the given color mask
                X is positive to the right of the neato, Y is positive in front of the neato, and height is ignored,
                or None if no significant amount of the given color is present in the image
        """
        # Calculate the color mask
        mask = cv2.inRange(cv_image, lower_mask, upper_mask)
        if SHOWVISUALS:
            cv2.imshow(f'{color} mask',mask)
            k=cv2.waitKey(10)
            if k==27:
                cv2.destroyAllWindows()

        # Find the outline of the blobs in the color mask
        blobs, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        if len(blobs) == 0:
            return None
        blobs = [b.reshape((-1, 2)) for b in blobs]

        # Estimate the blob as a rectangle and calculate the size of the largest one
        # We estimate it as a rectangle both for ease and because a line of sticky notes is a rectangle
        blob_sizes = [(np.max(b[:, 0]) - np.min(b[:, 0])) * (np.max(b[:, 1]) - np.min(b[:, 1])) for b in blobs]
        big_blob_idx = max(range(len(blobs)), key=lambda i: blob_sizes[i])
        if blob_sizes[big_blob_idx] / mask.size < 0.002:
            # If the largest blob is less than 0.2% of the image, ignore it, return None
            return None
        big_blob = blobs[big_blob_idx]

        # Find the dimensions of the largest blob
        blob_center_pix = np.mean(big_blob, axis=0)+0.5 #.5 added to account for pixel center
        blob_max_y = np.max(big_blob[:, 1])
        blob_min_y = np.min(big_blob[:, 1])
        blob_height_pix = blob_max_y - blob_min_y
        if blob_height_pix == 0:
            # If we only detect a line, ignore this mask, return None
            return None
        # print(blob_center_pix, blob_height_pix)

        # Calculate the 3D position of the blob
        # (possible given camera intrinsics and the height of a sticky note in reality)
        z_distance = FOCAL_Y*NOTE_HEIGHT/blob_height_pix
        center_homogenous = np.array([blob_center_pix[0], blob_center_pix[1], 1])
        dir_3d = np.linalg.inv(CALIBRATION_MATRIX) @ center_homogenous
        point_in_3D = z_distance * dir_3d / dir_3d[2]

        # Convert to particle and return
        pose = Pose()
        # print(point_in_3D)
        pose.position.x = point_in_3D[0]
        pose.position.y = point_in_3D[2]
        return Particle(pose=pose, weight=1.0)


def main():
    rclpy.init()
    n = CameraDetector()
    rclpy.spin(n)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
