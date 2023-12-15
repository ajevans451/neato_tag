import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from neato2_interfaces.msg import Bump
from std_msgs.msg import Int8
import cv2
import numpy as np
from neato_tag.game import NUM_PIXELS, NEATO_TAG, COLOR_TO_MASK
from cv_bridge import CvBridge


SHOW_VISUALS = False


class Tagging(Node):
    def __init__(self):
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
        self.is_it = it_msg.data == self.neato_id
    
    def find_bump(self, bump_msg: Bump):
        neato = self.closest_neato
        if self.is_it and (bump_msg.left_front or bump_msg.left_side or bump_msg.right_front or bump_msg.right_side) and neato is not None:
            msg = Int8()
            msg.data = neato
            self.it_pub.publish(msg)
    
    def find_closest_neato(self, image_msg: Image):
        cv_image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        if SHOW_VISUALS:
            cv2.imshow('frame', cv_image)
            k=cv2.waitKey(10)
            if k==27:
                cv2.destroyAllWindows()

        color_sizes = [0 for _ in range(NEATO_TAG.num_players)]
        for idx, color in enumerate(NEATO_TAG.player_colors):
            mask = COLOR_TO_MASK[color]
            masked_image = cv2.inRange(hsv_image, mask[0], mask[1])
            if SHOW_VISUALS:
                cv2.imshow(color.lower(), masked_image)
                k=cv2.waitKey(10)
                if k==27:
                    cv2.destroyAllWindows()
            blobs, _ = cv2.findContours(masked_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            if len(blobs) == 0:
                continue
            blobs = [b.reshape((-1, 2)) for b in blobs]
            blob_sizes = [(np.max(b[:, 0]) - np.min(b[:, 0])) * (np.max(b[:, 1]) - np.min(b[:, 1])) for b in blobs]
            big_blob_size = max(blob_sizes)
            if big_blob_size / masked_image.size > 0.02:
                color_sizes[idx] = big_blob_size
        
        max_idx = max(range(len(color_sizes)), key=lambda i: color_sizes[i])
        if color_sizes[max_idx] == 0:
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
