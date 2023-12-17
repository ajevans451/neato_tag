import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from neato2_interfaces.msg import Bump
from std_msgs.msg import Int8
import cv2
import numpy as np
from neato_tag.game import NEATO_TAG, COLOR_TO_MASK
from cv_bridge import CvBridge


SHOW_VISUALS = False


class Tagging(Node):
    """
    A Node to determine when the neato running it has tagged another neato
    """
    def __init__(self):
        """
        Create a new instance of the Tagging node
        """
        super().__init__('tagging')
        self.create_subscription(Image, 'camera/image_raw', self.find_closest_neato, 10)
        self.create_subscription(Bump, 'bump', self.find_bump, 10)
        self.create_subscription(Int8, 'it_status', self.update_it, 10)
        self.it_pub = self.create_publisher(Int8, 'it_status', 10)

        self.neato_id = self.declare_parameter('neato_id', 0).value
        self.is_it = self.declare_parameter('start_as_it', True).value
        self.closest_neato = None
        self.bridge = CvBridge()
        
        if SHOW_VISUALS:
            cv2.namedWindow('frame')
            for color in NEATO_TAG.player_colors:
                cv2.namedWindow(color.lower())
    
    def update_it(self, it_msg: Int8):
        """
        Callback function for the it_status topic

        Sets it_status depending on if this neato was just tagged

        Args:
            it_msg: an Int8 message giving the ID of the last neato tagged
        """
        self.is_it = it_msg.data == self.neato_id
    
    def find_bump(self, bump_msg: Bump):
        """
        Callback function for the bump topic

        If the bump sensor was triggered and this neato is it, send a tag message on it_status
        corresponding to the closest neato

        Args:
            bump_msg: a Bump message giving the status of the four bump sensors
        """
        neato = self.closest_neato
        was_bumped = bump_msg.left_front or bump_msg.left_side or bump_msg.right_front or bump_msg.right_side
        if self.is_it and was_bumped and neato is not None:
            msg = Int8()
            msg.data = neato
            self.it_pub.publish(msg)
    
    def find_closest_neato(self, image_msg: Image):
        """
        Callback function for camera/image_raw topic

        Find the closest neato from all neatos playing the game

        Args:
            image_msg: the Image message from camera/image_raw
        """
        # Convert image to HSV
        cv_image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        if SHOW_VISUALS:
            cv2.imshow('frame', cv_image)
            k=cv2.waitKey(10)
            if k==27:
                cv2.destroyAllWindows()

        # For each color, find the size of the largest blob of that color
        color_sizes = [0 for _ in range(NEATO_TAG.num_players)]
        for idx, color in enumerate(NEATO_TAG.player_colors):
            # Mask the image based on the color
            mask = COLOR_TO_MASK[color]
            masked_image = cv2.inRange(hsv_image, mask[0], mask[1])
            if SHOW_VISUALS:
                cv2.imshow(color.lower(), masked_image)
                k=cv2.waitKey(10)
                if k==27:
                    cv2.destroyAllWindows()

            # Find the blobs in the masked image
            blobs, _ = cv2.findContours(masked_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            if len(blobs) == 0:
                # If no blobs found, leave color_sizes[idx] as 0
                continue
            blobs = [b.reshape((-1, 2)) for b in blobs]

            # Find the size of the largest blob
            blob_sizes = [(np.max(b[:, 0]) - np.min(b[:, 0])) * (np.max(b[:, 1]) - np.min(b[:, 1])) for b in blobs]
            big_blob_size = max(blob_sizes)
            if big_blob_size / masked_image.size > 0.02:
                # Only set color_sizes[idx] if blob size is significant - at least 2% of image
                color_sizes[idx] = big_blob_size

        # Find the largest blob and set closest_neato accordingly
        max_idx = max(range(len(color_sizes)), key=lambda i: color_sizes[i])
        if color_sizes[max_idx] == 0:
            # If no color hit "large enough", no closest neato
            self.closest_neato = None
            return
        self.closest_neato = max_idx


def main():
    rclpy.init()
    node = Tagging()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
