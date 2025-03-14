import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd 


class OptimizedGapFollower(Node):
    def __init__(self):
        super().__init__('optimized_gap_follower')
        
        # Parameters
        self.max_speed = 2.0
        self.max_angle_vel = 1.5
        self.r_b = 0.12  # Safety bubble (inflation) radius in meters
        self.t = 1.5    # Threshold distance: any reading below this is considered a near obstacle
        self.max_range = 7.0  # Maximum sensor reading (used to replace inf values)
        
        self.robot_yaw = 0.0          # Current yaw (from odometry)
        self.latest_heading_error = 0.0
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.positions = []  # To store (x, y) positions
        self.world_size = 9  # Define the world size
        
        # For gradual angular velocity update:
        self.current_angular_vel = 0.0  # Current commanded angular velocity
        self.angular_step = 0.6         # Step size for changing angular velocity
        
        # Subscribers & Publishers
        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 1)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 1)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 1)
    
    def odom_callback(self, msg):
        q = msg.pose.pose.orientation
        # Compute yaw from quaternion (only yaw needed for 2D navigation)
        _, _, self.robot_yaw = self.euler_from_quaternion(q.x, q.y, q.z, q.w)
        self.get_logger().info(f"Current robot yaw: {self.robot_yaw:.2f}")
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.positions.append((x, y))

    def euler_from_quaternion(self, x, y, z, w):
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return 0.0, 0.0, yaw

    def lidar_callback(self, scan_msg):
        """
        Process LiDAR data by restricting attention to the front field of view, marking any beam
        that is too close to an obstacle (and a safety window around it) as invalid, and then selecting
        the beam with the maximum original range among those with distance < 6.0 meters.
        """
        # Get LiDAR ranges and replace infinite values with max_range.
        ranges = np.array(scan_msg.ranges)
        ranges = np.where(np.isinf(ranges), self.max_range, ranges)
        
        # Since the LiDAR measures 360° starting at 0 (front) and increasing counterclockwise,
        # we consider only the front field of view (0° to 90° and 270° to 360°).
        total_beams = len(ranges)
        degrees_per_beam = 360.0 / total_beams
        
        # Indices for 0° to 90°
        index_90 = int(90 / degrees_per_beam)
        # Indices for 270° to 360°
        index_270 = int(270 / degrees_per_beam)
        
        # Create a boolean mask: set True for beams in the front FOV.
        front_mask = np.zeros_like(ranges, dtype=bool)
        front_mask[0:index_90 + 1] = True
        front_mask[index_270:total_beams] = True
        
        # Select only the front beams.
        ranges_front = ranges[front_mask]
        
        # Copy original front ranges for inflation processing.
        inflated_ranges = ranges_front.copy()
        angle_inc = scan_msg.angle_increment  # Angular increment in radians

        # For every beam in the front FOV with a range below the safety threshold,
        # zero out a window around it.
        for i, r in enumerate(ranges_front):
            if r < self.t:
                # Compute the inflation angle: the safety bubble subtends an angle given by arcsin(r_b / r).
                # If the reading is extremely close (r < r_b), use a wide window.
                if r > self.r_b:
                    inflation_angle = np.arcsin(self.r_b / r)
                else:
                    inflation_angle = np.pi / 2  # maximum inflation if too close
                window = int(np.ceil(inflation_angle / angle_inc))
                start = max(0, i - window)
                end = min(len(ranges_front) - 1, i + window)
                inflated_ranges[start:end+1] = 0.0

        # Consider all beams with non-zero inflated range as valid.
        valid_indices = np.where(inflated_ranges > 0)[0]
        if valid_indices.size == 0:
            self.get_logger().warn("No valid path found! Stopping robot.")
            twist_msg = Twist()
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.0
            self.cmd_pub.publish(twist_msg)
            return

        # Filter valid beams with a distance less than 6.0 meters.
        valid_less_than_6 = valid_indices[ranges_front[valid_indices] < 7.0]
        if valid_less_than_6.size > 0:
            best_index = valid_less_than_6[np.argmax(ranges_front[valid_less_than_6])]
        else:
            # Fallback: choose the beam with the maximum original range among all valid beams.
            best_index = valid_indices[np.argmax(ranges_front[valid_indices])]
        
        # Map the best_index (from ranges_front) back to the original indices.
        original_indices = np.where(front_mask)[0]
        absolute_best_index = original_indices[best_index]
        
        # Compute the best heading in radians.
        best_heading = scan_msg.angle_min + absolute_best_index * angle_inc
        selected_distance = ranges_front[best_index]
        self.get_logger().info(
            f"Selected beam index: {absolute_best_index}, heading: {best_heading:.2f} rad, "
            f"distance: {selected_distance:.2f} m"
        )
        
        # Compute heading error (assuming LiDAR's angle_min aligns with the robot’s forward direction).
        heading_error = best_heading - 0.01  # applying a small offset (noise correction)
        self.latest_heading_error = heading_error

        # Adjust linear speed: reduce speed on sharper turns.
        if abs(heading_error) > 0.3:
            linear_speed = self.max_speed * 0.7
        else:
            linear_speed = self.max_speed

        self.update_robot_movement(linear_speed)

    def steering_control(self, heading_error, k_p=0.7):
        """
        Compute angular velocity from heading error using a proportional controller,
        ensuring that angles in [π, 2π] become negative (so the robot turns to the right).
        """
        # Normalize heading_error to [-pi, pi].
        if heading_error > math.pi:
            heading_error -= 2.0 * math.pi
        elif heading_error < -math.pi:
            heading_error += 2.0 * math.pi
        
        # Compute the proportional steering command.
        angular_vel = k_p * heading_error
        
        # Clip the angular velocity to the maximum allowed value.
        angular_vel = max(min(angular_vel, self.max_angle_vel), -self.max_angle_vel)
        
        return angular_vel

    
    def update_robot_movement(self, linear_speed):
        """
        Gradually update the angular velocity toward the target while publishing the command.
        """
        target_angular_vel = self.steering_control(self.latest_heading_error)
        # Smooth the angular velocity update for stability at high speeds.
        if self.current_angular_vel < target_angular_vel:
            self.current_angular_vel = min(self.current_angular_vel + self.angular_step, target_angular_vel)
        elif self.current_angular_vel > target_angular_vel:
            self.current_angular_vel = max(self.current_angular_vel - self.angular_step, target_angular_vel)
        
        twist_msg = Twist()
        twist_msg.linear.x = linear_speed
        twist_msg.angular.z = self.current_angular_vel
        self.cmd_pub.publish(twist_msg)
        self.get_logger().info(
            f"Command: linear.x={linear_speed:.2f}, angular.z={self.current_angular_vel:.2f}"
        )

    def save_path_plot(self):
        """Saves and prints the robot’s path and compares it with the best path."""
        best_path_df = pd.read_csv("/home/zaynap/project_ws/trajectory.csv")  

        if not self.positions:
            self.get_logger().warn("No positions found, not saving.")
            return

        # Extract actual path
        x_vals, y_vals = zip(*self.positions)
        
        # Extract best path from CSV
        best_x = best_path_df["x"].tolist()
        best_y = best_path_df["y"].tolist()
        actualpath = list(zip(x_vals, y_vals))  # actual path 
        refernce_path = list(zip(best_x, best_y))
        smoothness = self.calculate_smoothness(actualpath)
        deviations = self.calculate_deviation(actualpath, refernce_path)
        self.plot_results(actualpath, refernce_path, smoothness, deviations)


        # Print both paths for debugging
        # print("Actual Path:", list(zip(x_vals, y_vals)))
        # print("Best Path:", list(zip(best_x, best_y)))

        # Compute RMS error
#         actual = np.array(self.positions)
#         optimal = np.array(list(zip(best_x, best_y)))


# # Ensure both have the same length
#         min_length = min(len(actual), len(optimal))
#         actual = actual[:min_length]
#         optimal = optimal[:min_length]

#         rms_error = np.sqrt(np.mean((actual - optimal) ** 2))

#         # Plot actual vs best path
#         plt.figure(figsize=(6, 6))
#         plt.plot(y_vals, x_vals, marker='o', markersize=2, linestyle='-', color='blue', label="Actual Path")
#         plt.plot(best_y, best_x, marker='s', markersize=2, linestyle='--', color='red', label="Best Path")
#         plt.xlim(22,-8)
#         plt.ylim(30 , -3 )
#         plt.xlabel("Y Axis")
#         plt.ylabel("X Axis (Vertical)")
#         plt.title(f"Robot Path (RMS Error: {rms_error:.3f})")

#         plt.legend()
#         plt.grid()
#         plt.gca().invert_yaxis()  # Match world view perspective

#         # Save plot
#         plt.savefig("/home/zaynap/project_ws/trajectory_comparison.png")
#         plt.show()

#         self.get_logger().info(f"Saved trajectory plot with RMS error: {rms_error:.3f}")

    def calculate_smoothness(self, path):
        angles = []
        for i in range(1, len(path) - 1):
            v1 = np.array(path[i]) - np.array(path[i - 1])
            v2 = np.array(path[i + 1]) - np.array(path[i])
            dot_product = np.dot(v1, v2)
            norm_product = np.linalg.norm(v1) * np.linalg.norm(v2)
            if norm_product == 0:
                angles.append(0)
            else:
                angle = np.arccos(np.clip(dot_product / norm_product, -1.0, 1.0))
                angles.append(angle)
        return angles


    def calculate_deviation(self ,path, reference_path):
        deviations = []
        for point in path:
            distances = [np.linalg.norm(np.array(point) - np.array(ref_point)) for ref_point in reference_path]
            deviations.append(min(distances))
        return deviations
    
    def plot_results(self ,path, reference_path, smoothness, deviations):
        path = np.array(path)
        reference_path = np.array(reference_path)
        
        rmse = np.sqrt(np.mean(np.array(deviations) ** 2))
        avg_deviation = np.mean(deviations)
        
        # Plot path
        plt.figure(figsize=(10, 5))
        plt.plot(reference_path[:, 1], reference_path[:, 0], 'g--', label="Reference Path")
        plt.plot(path[:, 1], path[:, 0], 'b-', label="Actual Path")
        plt.xlabel("y-axis")
        plt.ylabel("x-axis")
        plt.title("Path with RMSE and Average Deviation")
        plt.legend()
        plt.text(path[0, 0], path[0, 1], f'RMSE: {rmse:.4f}\nAvg Dev: {avg_deviation:.4f}', fontsize=10, bbox=dict(facecolor='white', alpha=0.5))
        plt.gca().invert_xaxis()  # Match world view perspective

        plt.savefig("/home/zaynap/project_ws/path_plotOgapF.png")
        # plt.show()
        
        # Plot smoothness and deviation
        plt.figure(figsize=(10, 5))
        plt.plot(range(len(smoothness)), smoothness, label="Smoothness")
        plt.plot(range(len(deviations)), deviations, label="Deviation")
        plt.xlabel("Point Index")
        plt.ylabel("Value")
        plt.title("Smoothness and Standard Deviation along the Path")
        plt.legend()

        plt.savefig("/home/zaynap/project_ws/smoothness_and_deviationOgapF.png")
        plt.show()

def main(args=None):
    rclpy.init(args=args)
    node = OptimizedGapFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # node.publish_stop()  # Ensure the robot stops before shutdown.
        node.save_path_plot()  # Save the path when stopping
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
