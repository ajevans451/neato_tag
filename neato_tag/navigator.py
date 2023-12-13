import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8
import numpy as np
from nav2_msgs.msg import ParticleCloud
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

FORWARD_ALPHA = 1000.0
LIDAR_ALPHA = 1
CAMERA_ALPHA = 1000
TURN_CONSTANT=5
FORWARD_CONSTANT = 0.15
neato_id=1

class Navigator(Node):

    def __init__(self,neato_id):
        super().__init__('navigator')
        self.camera_subscriber = self.create_subscription(ParticleCloud, 'neatos_in_camera', self.camera_callback, 10)
        self.it_status_subscriber = self.create_subscription(Int8, 'it_status', self.check_it_status, 10)
        self.lidar_subscriber = self.create_subscription(LaserScan, 'scan', self.avoid_obstacles, 10)
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.lidar_gradient = np.zeros((2,))
        self.cam_gradient = np.zeros((2,))
        self.it_status = False
        self.neato_id = neato_id
        self.create_timer(0.03, self.run_loop)

    def run_loop(self):
        drive_msg = Twist()
        forward_gradient = np.array([FORWARD_ALPHA / 500, 0])
        if self.it_status:
            final_gradient = self.cam_gradient + self.lidar_gradient + forward_gradient
        else:
            final_gradient = -self.cam_gradient + self.lidar_gradient - forward_gradient
        target_distance = np.hypot(final_gradient[0], final_gradient[1])
        target_angle = -np.arctan2(final_gradient[1], final_gradient[0])
        
        if not self.it_status:
            target_distance *= -1
            target_angle = np.pi - target_angle
            if target_angle > np.pi:
                target_angle -= 2*np.pi
            target_angle *= -1
        print(target_angle, target_distance)
        
        drive_msg.linear.x = np.clip(float(FORWARD_CONSTANT * target_distance), -1.0, 1.0)    #Determines final linear drive message
        drive_msg.angular.z = np.clip(float(TURN_CONSTANT * target_angle), -1.0, 1.0)         #Determines final angular drive message
        self.pub.publish(drive_msg) #Commands Neato motors
    
    def check_it_status(self,msg):
        if msg.data==self.neato_id:
            self.it_status=True
        else:
            self.it_status=False

    def camera_callback(self,msg):
        cart_x=[]
        cart_y=[]
        for particle in msg.particles:
            # yes x and y are switched
            cart_x.append(particle.pose.position.y)
            cart_y.append(particle.pose.position.x)
        cart_x=np.array(cart_x)
        cart_y=np.array(cart_y)
        dist_sq = cart_x ** 2 + cart_y ** 2
        cartesian = np.stack([cart_x, cart_y])
        source_deriv = 2 * cartesian / (dist_sq ** 2) / cartesian.shape[1] * CAMERA_ALPHA
        self.cam_gradient = np.sum(source_deriv, axis=1)

    def avoid_obstacles(self,msg):
        angles = np.arange(msg.angle_min, msg.angle_max, msg.angle_increment)
        # Flip angles around the y axis b/c straight forward is 180??
        angles = np.sign(angles) * np.pi - angles + np.pi * (angles == 0)
        ranges = np.array(msg.ranges)
        points = np.stack([angles,ranges])
        point_filter = (ranges >= msg.range_min) & (ranges <= msg.range_max)
        filtered_points = points[:, point_filter]

        cart_x = filtered_points[1, :] * np.cos(filtered_points[0, :])
        cart_y = filtered_points[1, :] * np.sin(filtered_points[0, :])
        cartesian = np.stack([cart_x, cart_y])
        
        dist_sq = cart_x ** 2 + cart_y ** 2
        source_deriv = 2 * cartesian / (dist_sq ** 2) / cartesian.shape[1] * LIDAR_ALPHA
        self.lidar_gradient = -np.sum(source_deriv, axis=1)
        

def main(args=None):
    rclpy.init()
    n = Navigator(1)
    rclpy.spin(n)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
