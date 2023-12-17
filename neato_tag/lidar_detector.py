
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav2_msgs.msg import ParticleCloud, Particle
from geometry_msgs.msg import Pose
import numpy as np


NEATO_RADIUS = 0.125  # meters
MIN_RADIUS = 0.9 * NEATO_RADIUS
MAX_RADIUS = 1.1 * NEATO_RADIUS


def pose_from_point(x: float, y: float) -> Pose:
    """
    Convert an x/y position into a Pose

    Args:
        x: a float, the x coordinate
        y: a float, the y coordinate
    
    Returns:
        a Pose with the given x and y coordinates, and no particular orientation (z is 0)
    """
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    return pose


class LidarDetector(Node):
    """
    Given a LiDAR scan, find other neatos (arcs with a radius of NEATO_RADIUS)
        and publish of the neato in current-neato frame
    """
    def __init__(self):
        """
        Create a new instance of the lidar detector node
        """
        super().__init__('lidar_detector')
        self.lidar_subscriber = self.create_subscription(LaserScan, 'scan', self.find_neatos, 10)
        self.publisher = self.create_publisher(ParticleCloud, 'neatos_in_lidar', 10)
    
    def find_neatos(self, scan: LaserScan):
        """
        Callback for the scan; given a LiDAR scan, find neatos within the scan and publish
            their locations to neatos_in_lidar

        Args:
            scan: the LaserScan message given by the neato's LiDAR
        """
        # Convert the angles and distances to numpy arrays
        angles = np.arange(scan.angle_min, scan.angle_max, scan.angle_increment)
        dists = np.array(scan.ranges)
        is_in_range = (dists > scan.range_min) & (dists < scan.range_max)
        npoints = dists.shape[0]
        points = []  # to add to point cloud
        cartesian_points = np.stack((np.cos(angles) * dists, np.sin(angles) * dists))

        # For each point, see if it is a neato
        for idx, (angle, dist) in enumerate(zip(angles, dists)):
            if idx == 0 or idx == npoints - 1 or not is_in_range[idx]:
                continue
            # Find the nearest point in range to the left and right of current point
            idx_to_left = idx - 1
            while not is_in_range[idx_to_left]:
                idx_to_left -= 1
                if idx_to_left == -1:
                    break
            idx_to_right = idx + 1
            while not is_in_range[idx_to_right]:
                idx_to_right += 1
                if idx_to_right == npoints:
                    idx_to_right = -1
                    break
            if idx_to_left == -1 or idx_to_right == -1:
                continue

            # Find the X/Y coordinates of the three points
            lft = cartesian_points[0, idx_to_left] + cartesian_points[1, idx_to_left] * 1j
            rht = cartesian_points[0, idx_to_right] + cartesian_points[1, idx_to_right] * 1j
            ctr = cartesian_points[0, idx] + cartesian_points[1, idx] * 1j

            # Find the center and radius of a circle going through the three points
            # Code adopted from https://stackoverflow.com/questions/28910718/give-3-points-and-a-plot-circle
            x, y, z = lft, rht, ctr
            w = z-x
            w /= y-x
            c = (x-y)*(w-abs(w)**2)/2j/w.imag-x
            circle_center = -c
            circle_center_arr = np.array([circle_center.real, circle_center.imag])
            circle_radius = abs(c + x)

            if not MIN_RADIUS <= circle_radius <= MAX_RADIUS:
                # If the circle isn't the right size to be a neato, ignore
                continue

            # Find the points which are part of the circle
            dists_to_center = np.sqrt(np.sum((cartesian_points.T - circle_center_arr) ** 2, axis=1))
            close_points = (dists_to_center >= MIN_RADIUS) & (dists_to_center <= MAX_RADIUS)
            num_close_pts = np.sum(close_points)
            if num_close_pts < 10:
                # If there are less than 10 points as part of the circle, ignore
                continue

            # Remove the points in the circle (no double counting
            is_in_range[close_points] = False

            # Append the circle to the list of points
            points.append((circle_center.real, circle_center.imag))
            # print(circle_radius)

        # Create the ParticleCloud message and publish it
        msg = ParticleCloud()
        msg.header = scan.header
        msg.particles.extend(Particle(pose=pose_from_point(*pt), weight=1.0) for pt in points)
        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = LidarDetector()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
