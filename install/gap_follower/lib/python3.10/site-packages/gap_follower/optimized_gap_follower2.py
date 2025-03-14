import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import numpy as np

class OptimizedGapFollower(Node):
    def __init__(self):
        super().__init__('optimized_gap_follower')
        
        # Parameters
        self.max_speed = 2.0
        self.max_angle_vel = 1.5
        self.r_b = 0.2   # Safety bubble (inflation) radius in meters
        self.n = 30      # Minimum gap size (in number of LiDAR beams)
        self.t = 1.3  # Threshold distance for obstacle inflation
        self.max_range = 3.5  # Maximum sensor reading (used to replace inf values)
        
        self.robot_yaw = 0.0          # Current yaw (from odometry)
        self.latest_heading_error = 0.0
        
        # For gradual angular velocity update:
        self.current_angular_vel = 0.0  # Current commanded angular velocity
        self.angular_step = 0.6        # Step size for changing angular velocity
        
        # Subscribers & Publishers
        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 1)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 1)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 1)
    
    def odom_callback(self, msg):
        q = msg.pose.pose.orientation
        # Compute yaw from quaternion (only yaw needed for 2D navigation)
        _, _, self.robot_yaw = self.euler_from_quaternion(q.x, q.y, q.z, q.w)
        self.get_logger().info(f"Current robot yaw: {self.robot_yaw:.2f}")

    def euler_from_quaternion(self, x, y, z, w):
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return 0.0, 0.0, yaw

    def lidar_callback(self, scan_msg):
        """Process LiDAR data, inflate obstacles, and select the best gap."""
        # Use a subset of LiDAR data (e.g. indices 0 to 180)
        self.get_logger().info(f"Current robot yaw: {self.robot_yaw:.2f}")

        ranges = np.array(list(scan_msg.ranges))
        total_rays = len(ranges)
        self.get_logger().info(f"total rays : {total_rays: .2f}")
        start_idx = 0
        # end_idx = total_rays // 2 
        end_idx= total_rays
        ranges = ranges[start_idx:end_idx + 1]
        angle_min = scan_msg.angle_min
        angle_inc = scan_msg.angle_increment
        
        # Replace infinite readings with self.max_range
        ranges = np.where(np.isinf(ranges), self.max_range, ranges)
        ranges[90:270] = 1.0
        self.get_logger().warn(f"Raw LiDAR data: {ranges}")

        
        # Copy original ranges to effective_ranges for modification.
        effective_ranges = ranges.copy()
        # Create a mask to track beams affected by inflation.
        inflated_mask = np.zeros_like(ranges, dtype=int)
        
        # Compute sensor beam angles for this subset.
        sensor_angles = np.array([angle_min + i * angle_inc for i in range(len(ranges))])
        
        # For each beam with a reading below the threshold,
        # treat it as an obstacle and compute its inflation intersection on all sensor beams.
        for i, r in enumerate(ranges):
            if r < self.t:
                # Obstacle center in polar coordinates for this beam.
                r_obs = r
                theta_obs = sensor_angles[i]
                # For every sensor beam, compute the intersection with the inflated circle.
                for j, theta in enumerate(sensor_angles):
                    infl_range = self.inflated_intersection(r_obs, theta_obs, self.r_b, theta)
                    # If an inflated intersection is closer than current effective range, update.
                    if infl_range < effective_ranges[j]:
                        effective_ranges[j] = infl_range
                        inflated_mask[j] = 1
        
        # Mark a beam as free (1) only if no inflation intersection occurred.
        free_space = np.where(inflated_mask == 0, 1, 0)
        self.get_logger().info(f"the free space is: {free_space} ")
        
        # Now, extract gaps from the free_space mask.
        gap_start, gap_end = self.find_largest_gap(free_space)
        if gap_start is None or gap_end is None:
            gap_start, gap_end = self.find_largest_available_gap(free_space)
        
        self.get_logger().info(f"Selected Gap: Start={gap_start}, End={gap_end}")

        
        if gap_start is None or gap_end is None:
            self.get_logger().warn("No free gap found! Stopping robot.")
            twist_msg = Twist()
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.0
            self.cmd_pub.publish(twist_msg)
            return
        
        # Select the best heading (using the midpoint of the gap).
        best_heading = self.select_best_point(gap_start, gap_end, angle_min, angle_inc, effective_ranges)
        # if best_heading > math.pi:
        if best_heading > 3.14:
            best_heading -= 2 * math.pi

        self.get_logger().info(f"Selected heading: {best_heading:.2f} rad")
        # heading_error=best_heading - 1.6
        # heading_error = 1.57 - best_heading

        heading_error = best_heading
        # heading_error = heading_error - self.robot_yaw
        # heading_error = self.robot_yaw - heading_error
        self.latest_heading_error = heading_error
        
        self.get_logger().info(f"Selected heading: {best_heading:.2f} rad, Heading error: {heading_error:.2f}")
        self.update_robot_movement()
    
    def inflated_intersection(self, r_obs, theta_obs, r_b, theta):
        """
        For an obstacle at (r_obs, theta_obs) with inflation radius r_b,
        compute the intersection distance along a sensor ray at angle theta.
        Returns np.inf if the ray does not intersect the inflated circle.
        """
        delta = theta - theta_obs
        if abs(r_obs * np.sin(delta)) > r_b:
            return np.inf  # No intersection.
        try:
            r_inflated = r_obs * np.cos(delta) - np.sqrt(r_b**2 - (r_obs * np.sin(delta))**2)
        except ValueError:
            return np.inf
        return r_inflated if r_inflated > 0 else np.inf

    def extract_gaps(self, free_space):
        """Extract continuous segments (gaps) in the free_space mask."""
        gaps = []
        start = None
        for i in range(len(free_space)):
            if free_space[i] == 1:
                if start is None:
                    start = i
            else:
                if start is not None:
                    gaps.append((start, i - 1))
                    start = None
        if start is not None:
            gaps.append((start, len(free_space) - 1))
        return gaps

    def find_largest_gap(self, free_space):
        """Return the largest gap that meets the minimum size requirement."""
        gaps = self.extract_gaps(free_space)
        valid_gaps = [gap for gap in gaps if (gap[1] - gap[0] + 1) >= self.n]
        return max(valid_gaps, key=lambda g: g[1] - g[0]) if valid_gaps else (None, None)

    def find_largest_available_gap(self, free_space):
        """Return the largest gap even if it is smaller than the minimum required size."""
        gaps = self.extract_gaps(free_space)
        return max(gaps, key=lambda g: g[1] - g[0], default=(None, None))
    
    def select_best_point(self, gap_start, gap_end, angle_min, angle_inc, ranges):
        """
        Select the best heading from the gap.
        Here, we choose the midpoint of the gap.
        """
        # mid_gap_idx = (gap_start + gap_end) // 2
        max_range_idx = max(range(gap_start, gap_end + 1), key=lambda i: ranges[i])


        best_heading = angle_min + (max_range_idx * angle_inc)
        return best_heading
    
    def steering_control(self, heading_error, k_p=0.3):
        """
        Compute angular velocity from heading error.
        Dynamically adjust the proportional gain.
        """
        k_p = 0.7 if abs(heading_error) > 0.5 else 0.5
        angular_vel = k_p * heading_error
        return max(min(angular_vel, self.max_angle_vel), -self.max_angle_vel)
    
    def update_robot_movement(self):
        """
        Gradually update the angular velocity toward the target heading.
        """
        target_angular_vel = self.steering_control(self.latest_heading_error)
        if self.current_angular_vel < target_angular_vel:
            self.current_angular_vel = min(self.current_angular_vel + self.angular_step, target_angular_vel)
        elif self.current_angular_vel > target_angular_vel:
            self.current_angular_vel = max(self.current_angular_vel - self.angular_step, target_angular_vel)
        
        twist_msg = Twist()
        twist_msg.linear.x = self.max_speed
        twist_msg.angular.z = self.current_angular_vel
        self.cmd_pub.publish(twist_msg)
        self.get_logger().info(
            f"Updated cmd_vel: linear.x={twist_msg.linear.x:.2f}, angular.z={twist_msg.angular.z:.2f}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = OptimizedGapFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
