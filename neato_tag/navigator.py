import rclpy
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Int8
import numpy as np
from nav2_msgs.msg import ParticleCloud
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


FORWARD_ALPHA = 500.0
LIDAR_ALPHA = 1
CAMERA_ALPHA_IT = 500
CAMERA_ALPHA_NOT_IT = 0.5
TURN_CONSTANT = 5
FORWARD_CONSTANT = 0.15
MAX_SPEED = 0.7
NOT_IT_SCALE = 0.9
FROZEN_TIME = 5 * 1e9  # nanoseconds


class Navigator(Node):
    """
    A Node to control the motion of the neatos as they play tag

    Attributes:
        camera_subscriber: the ROS Subscriber to the neatos_in_camera topic
        it_status_subscriber: the ROS Subscriber to the it_status topic
        lidar_subscriber: the ROS Subscriber to the scan topic
        pub: the ROS Publisher for the cmd_vel topic
        lidar_gradient: a 2D numpy vector, the ideal direction to go to avoid obstacles
        cam_gradient: a 2D numpy vector, the ideal direction to go to tag neatos
        neato_id: an int, the ID of the neato running this node
        it_status: a bool, True when this neato is it, False otherwise
        frozen_time: ROS Time specifying when the neato was frozen, or None when the neato is not frozen
    """
    def __init__(self):
        """
        Create a new Navigator node
        """
        super().__init__('navigator')
        self.camera_subscriber = self.create_subscription(ParticleCloud, 'neatos_in_camera', self.camera_callback, 10)
        self.it_status_subscriber = self.create_subscription(Int8, 'it_status', self.check_it_status, 10)
        self.lidar_subscriber = self.create_subscription(LaserScan, 'scan', self.avoid_obstacles, 10)
        self.create_timer(0.1, self.unfreeze)
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.lidar_gradient = np.zeros((2,))
        self.cam_gradient = np.zeros((2,))
        self.neato_id = self.declare_parameter('neato_id', 0).value
        self.it_status = self.declare_parameter('start_as_it', True).value
        self.frozen_time = None
        self.create_timer(0.03, self.run_loop)
    
    def unfreeze(self):
        """
        Checks if the neato's frozen time is up, and unfreezes it if it is
        """
        if self.frozen_time is None:
            return
        if (self.get_clock().now() - self.frozen_time).nanoseconds >= FROZEN_TIME:
            self.frozen_time = None

    def run_loop(self):
        """
        Main control loop for this neato

        Combines the gradients and controls the neato's motion
        """
        drive_msg = Twist()
        if self.frozen_time is not None:
            # If frozen, stop the neato and return
            self.pub.publish(drive_msg)
            return

        # Calculate the angle and speed the neato should travel
        forward_gradient = np.array([FORWARD_ALPHA / 500, 0]) # bias the neato forward (or backward) since it can't strafe
        if self.it_status:
            final_gradient = self.cam_gradient + self.lidar_gradient + forward_gradient
        else:
            final_gradient = -self.cam_gradient + self.lidar_gradient - forward_gradient
        target_distance = np.hypot(final_gradient[0], final_gradient[1])
        target_angle = -np.arctan2(final_gradient[1], final_gradient[0])

        # Fix the angle/dist if the neato is not it (runner neatos drive backwards)
        if not self.it_status:
            target_distance *= -1
            target_angle = np.pi - target_angle
            if target_angle > np.pi:
                target_angle -= 2*np.pi
            target_angle *= -1
        # print(target_angle, target_distance)

        # Calculate the linear and angular speeds
        lin_speed = np.clip(float(FORWARD_CONSTANT * target_distance), -MAX_SPEED, MAX_SPEED)
        ang_speed = np.clip(float(TURN_CONSTANT * target_angle), -MAX_SPEED, MAX_SPEED)
        if not self.it_status:
            lin_speed *= NOT_IT_SCALE
            ang_speed *= NOT_IT_SCALE

        # Send the drive message
        drive_msg.linear.x = lin_speed
        drive_msg.angular.z = ang_speed
        self.pub.publish(drive_msg) #Commands Neato motors
    
    def check_it_status(self, msg: Int8):
        """
        Callback function for it_status

        Sets it_status and frozen_time depending on if this neato has been tagged

        Args:
            msg: an Int8 message giving the ID of the neato that was just tagged
        """
        if msg.data==self.neato_id:
            self.it_status = True
            self.frozen_time = self.get_clock().now()
        else:
            self.it_status = False
            self.frozen = None

    def camera_callback(self, msg: ParticleCloud):
        """
        Callback function for neatos_in_camera

        Calculates cam_gradient based on the neatos found in the given message.
        cam_gradient moves the neato towards the neatos in the message

        Args:
            msg: the ParticleCloud published to neatos_in_camera
        """
        # Calculate the cartesian coordinates of the neatos
        cart_x=[]
        cart_y=[]
        for particle in msg.particles:
            # yes x and y are switched
            cart_x.append(particle.pose.position.y)
            cart_y.append(particle.pose.position.x)
        cart_x=np.array(cart_x)
        cart_y=np.array(cart_y)

        # Calculate the gradient at (0, 0)
        dist_sq = cart_x ** 2 + cart_y ** 2
        cartesian = np.stack([cart_x, cart_y])
        source_deriv = 2 * cartesian / (dist_sq ** 2) / cartesian.shape[1] * (CAMERA_ALPHA_IT if self.it_status else CAMERA_ALPHA_NOT_IT)
        self.cam_gradient = np.sum(source_deriv, axis=1)

    def avoid_obstacles(self, msg):
        """
        Callback function for scan

        Calculates lidar_gradient based on the obstacles found in the given message.
        cam_gradient moves the neato away from any point in the message

        Args:
            msg: the LaserScan published to scan
        """
        # Convert to numpy arrays
        angles = np.arange(msg.angle_min, msg.angle_max, msg.angle_increment)
        # Flip angles around the y axis b/c straight forward is 180??
        angles = np.sign(angles) * np.pi - angles + np.pi * (angles == 0)
        ranges = np.array(msg.ranges)
        points = np.stack([angles,ranges])

        # Filter out points which are out of range
        point_filter = (ranges >= msg.range_min) & (ranges <= msg.range_max)
        filtered_points = points[:, point_filter]

        # Convert remaining points to cartesian
        cart_x = filtered_points[1, :] * np.cos(filtered_points[0, :])
        cart_y = filtered_points[1, :] * np.sin(filtered_points[0, :])
        cartesian = np.stack([cart_x, cart_y])

        # Calculate the gradient at (0, 0)
        dist_sq = cart_x ** 2 + cart_y ** 2
        source_deriv = 2 * cartesian / (dist_sq ** 2) / cartesian.shape[1] * LIDAR_ALPHA
        self.lidar_gradient = -np.sum(source_deriv, axis=1)


def main():
    rclpy.init()
    n = Navigator()
    rclpy.spin(n)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
