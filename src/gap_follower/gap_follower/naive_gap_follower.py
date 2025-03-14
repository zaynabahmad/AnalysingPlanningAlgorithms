import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd 

class GapFollower(Node):
    def __init__(self):
        super().__init__('gap_follower')
        
        # Parameters
        self.max_speed = 1.5
        self.max_angle_vel = 0.8
        self.t = 2.5
        self.n = 30   
        self.robot_yaw = 0.0  
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.positions = []  # To store (x, y) positions
        self.world_size = 9  # Define the world size
        
        # Subscribers & Publishers
        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def odom_callback(self, msg):
        """Extracts position and yaw from odometry and stores it."""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.positions.append((x, y))
        
        # Limit stored positions to avoid excessive memory use
        if len(self.positions) > 1000:
            self.positions.pop(0)

    def euler_from_quaternion(self, x, y, z, w):
        """ Converts quaternion to roll, pitch, yaw """
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return 0.0, 0.0, yaw  

    def lidar_callback(self, scan_msg):
        """ Processes LiDAR data and follows the largest gap """
        ranges = list(scan_msg.ranges)
        total_rays = len(ranges)
        start_idx = 0  
        end_idx = total_rays // 2  

        filtered_ranges = ranges[start_idx:end_idx + 1]
        gap_start, gap_end = self.find_largest_gap(filtered_ranges)

        twist_msg = Twist()
        
        if gap_start is None or gap_end is None:
            self.get_logger().warn("No valid gap found! Stopping movement.")
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.0
        else:
            desired_heading = self.select_heading(gap_start, gap_end, scan_msg.angle_min, scan_msg.angle_increment, filtered_ranges)
            corrected_heading = desired_heading - self.robot_yaw
            corrected_heading = math.atan2(math.sin(corrected_heading), math.cos(corrected_heading))

            twist_msg.linear.x = self.max_speed  
            twist_msg.angular.z = self.steering_control(corrected_heading)

        self.cmd_pub.publish(twist_msg)

    def find_largest_gap(self, ranges):
        """ Finds the largest free space (gap) in LiDAR scan """
        gaps = []
        start = None
        for i in range(len(ranges)):
            if ranges[i] >= self.t:
                if start is None:
                    start = i
            else:
                if start is not None:
                    gaps.append((start, i - 1))
                    start = None
        if start is not None:
            gaps.append((start, len(ranges) - 1))
        
        valid_gaps = [gap for gap in gaps if (gap[1] - gap[0] + 1) >= self.n]
        if valid_gaps:
            largest_gap = max(valid_gaps, key=lambda g: g[1] - g[0])
            return largest_gap
        elif gaps:
            largest_gap = max(gaps, key=lambda g: g[1] - g[0])
            return largest_gap
        else:
            return None, None

    def select_heading(self, gap_start, gap_end, angle_min, angle_inc, ranges):
        """ Finds the heading angle towards the farthest point in the largest gap """
        max_range_idx = max(range(gap_start, gap_end + 1), key=lambda i: ranges[i])
        best_heading = angle_min + (max_range_idx * angle_inc)

        heading_error = best_heading - (math.pi / 2)
        heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))

        return heading_error

    def steering_control(self, heading_error, k_p=0.075):
        """ Converts heading error into angular velocity using proportional control """
        angular_vel = k_p * heading_error
        angular_vel = max(min(angular_vel, self.max_angle_vel), -self.max_angle_vel)
        return angular_vel

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

        plt.savefig("/home/zaynap/project_ws/path_plotNgapF.png")
        # plt.show()
        
        # Plot smoothness and deviation
        plt.figure(figsize=(10, 5))
        plt.plot(range(len(smoothness)), smoothness, label="Smoothness")
        plt.plot(range(len(deviations)), deviations, label="Deviation")
        plt.xlabel("Point Index")
        plt.ylabel("Value")
        plt.title("Smoothness and Standard Deviation along the Path")
        plt.legend()

        plt.savefig("/home/zaynap/project_ws/smoothness_and_deviationNgapF.png")
        plt.show()

def main(args=None):
    rclpy.init(args=args)
    node = GapFollower()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.save_path_plot()  # Save the path when stopping
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
