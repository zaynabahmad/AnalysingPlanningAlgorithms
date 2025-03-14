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
        self.max_speed = 1.6
        self.max_angle_vel = 1.0
        self.r_b = 0.5  # Safety bubble radius
        self.n = 25     # Minimum gap size
        self.robot_yaw = 0.0  # Current yaw from odometry
        self.latest_heading_error = 0.0
        
        # For gradual angular velocity update:
        self.current_angular_vel = 0.0  # Current commanded angular velocity
        self.angular_step = 0.5         # Step size for changing angular velocity
        
        # Subscribers & Publishers
        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        #  a timer that updates the velocity every 2 seconds.
        # Uncomment the timer if you wish to use gradual updates.
        # self.timer = self.create_timer(2.0, self.update_robot_movement)
    
    def odom_callback(self, msg):
        q = msg.pose.pose.orientation
        _, _, self.robot_yaw = self.euler_from_quaternion(q.x, q.y, q.z, q.w)
    
    def euler_from_quaternion(self, x, y, z, w):
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return 0.0, 0.0, yaw

    def lidar_callback(self, scan_msg):
        """Processes LiDAR data and follows the largest gap.""" 
        # try to get just from 0 to 180 ranges 
        ranges = list(scan_msg.ranges)
        total_rays = len(ranges)
        start_idx = 0  
        end_idx = total_rays // 2  

        filtered_ranges = ranges[start_idx:end_idx + 1]
        ranges = np.array(filtered_ranges)
        angle_min = scan_msg.angle_min
        angle_inc = scan_msg.angle_increment
        
        # Step 1: Apply Safety Bubble
        # nearest_idx = np.argmin(ranges)  # Index of nearest obstacle
        # nearest_dist = ranges[nearest_idx]  # Distance to nearest obstacle

        # get just the finite values 
        finite_mask = np.isfinite(ranges)
        if np.any(finite_mask):
            # Only consider finite measurements for finding the nearest obstacle.
            finite_ranges = ranges[finite_mask]
            # Get the index among the finite values.
            min_index_finite = np.argmin(finite_ranges)
            # Map this index back to the full array.
            finite_indices = np.nonzero(finite_mask)[0]
            nearest_idx = finite_indices[min_index_finite]
            nearest_dist = finite_ranges[min_index_finite]
        else:
            # If all values are infinite, assume no obstacle is detected.
            nearest_idx = None
            nearest_dist = np.inf

        # if nearest_dist < 2.0:  #  apply if the nearest object is closer than 2m
        #     safety_threshold = nearest_dist + self.r_b  # Define a clearance distance
        #     ranges[ranges <= safety_threshold] = 0  # Mark all points within this radius as obstacles  0 obstical , 1 free 

        # make it a buble not just a threshold 

        if nearest_idx is not None and nearest_dist < 2.0:
        # Calculate the angular width of the bubble using geometry:
        # For a bubble radius r_b and nearest distance d, the angle is:
        #    bubble_angle = arcsin(r_b/d)
            if nearest_dist > self.r_b:
                bubble_angle = math.asin(self.r_b / nearest_dist)
            else:
                bubble_angle = math.pi / 2  # If too close, remove a wide window.
        
            # Convert this angular width to an index offset.
            bubble_indices = int(bubble_angle / angle_inc)
            lower_bound = max(0, nearest_idx - bubble_indices)
            upper_bound = min(len(ranges) - 1, nearest_idx + bubble_indices)
            # Mark all points within the bubble as obstacles (set to 0).
            ranges[lower_bound:upper_bound + 1] = 0

        
        # Step 2: Identify Free Space
        free_space = np.where(ranges > 0, 1, 0)
        
        # Step 3: Find Largest Gap
        gap_start, gap_end = self.find_largest_gap(free_space)
        if gap_start is None or gap_end is None:
            self.get_logger().warn("No valid gap found! Looking for the largest available gap.")
            gap_start, gap_end = self.find_largest_available_gap(free_space)

        # Debug: Print detected gaps
        self.get_logger().info(f"Detected Gaps: {[(start, end) for start, end in self.extract_gaps(free_space)]}")

        twist_msg = Twist()
        if gap_start is None or gap_end is None:
            self.get_logger().warn("No available gap found! Stopping movement.")
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.0
            self.latest_heading_error = 0.0
        else:
            # Step 4: Select Best Point in the Largest Gap
            best_heading = self.select_best_point(gap_start, gap_end, angle_min, angle_inc, ranges)
            # Compute the heading error relative to the robot's yaw.
            # heeereeee
            heading_error = best_heading - self.robot_yaw
            heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))
            
            # Compute angular velocity  -->on steering control
            computed_ang_vel = self.steering_control(heading_error)
            twist_msg.linear.x = self.max_speed
            twist_msg.angular.z = computed_ang_vel
            
            # Save the latest heading error for gradual updates if desired.
            self.latest_heading_error = heading_error

        self.get_logger().info(f"cmd_vel (immediate): linear.x={twist_msg.linear.x:.2f}, angular.z={twist_msg.angular.z:.2f}")
        self.cmd_pub.publish(twist_msg)
    
    def find_largest_gap(self, free_space):
        gaps = self.extract_gaps(free_space)
        valid_gaps = [gap for gap in gaps if (gap[1] - gap[0] + 1) >= self.n]
        return max(valid_gaps, key=lambda g: g[1] - g[0]) if valid_gaps else (None, None)

    def find_largest_available_gap(self, free_space):
        """Finds the largest gap available, even if it's smaller than n."""
        gaps = self.extract_gaps(free_space)
        return max(gaps, key=lambda g: g[1] - g[0], default=(None, None))

    def extract_gaps(self, free_space):
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
    
    def select_best_point(self, gap_start, gap_end, angle_min, angle_inc, ranges):
        # mid point
        mid_gap_idx = (gap_start + gap_end) // 2
        best_heading = angle_min + (mid_gap_idx * angle_inc)
        # Return best_heading in radians directly.
        return best_heading

    def steering_control(self, heading_error, k_p=0.3):
        # dynamic gain daaa uta3adl
        k_p = 0.5 if abs(heading_error) > 0.5 else 0.3
        angular_vel = k_p * heading_error
        return max(min(angular_vel, self.max_angle_vel), -self.max_angle_vel)
    
    def update_robot_movement(self):  # da msh shaghal now 
        """
        Gradually update the angular velocity in steps.
        This callback is triggered every 2 seconds.
        """
        # Compute target angular velocity from the latest heading error.
        target_angular_vel = self.steering_control(self.latest_heading_error)
        
        # Gradually adjust the current angular velocity toward the target.
        if self.current_angular_vel < target_angular_vel:
            self.current_angular_vel = min(self.current_angular_vel + self.angular_step, target_angular_vel)
        elif self.current_angular_vel > target_angular_vel:
            self.current_angular_vel = max(self.current_angular_vel - self.angular_step, target_angular_vel)
        
        twist_msg = Twist()
        twist_msg.linear.x = self.max_speed
        twist_msg.angular.z = self.current_angular_vel
        self.cmd_pub.publish(twist_msg)
        
        self.get_logger().info(
            f"Updated cmd_vel (gradual): linear.x={twist_msg.linear.x:.2f}, angular.z={twist_msg.angular.z:.2f}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = OptimizedGapFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
