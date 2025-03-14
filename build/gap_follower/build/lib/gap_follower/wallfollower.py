import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np
import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry
import pandas as pd 


class WallFollower(Node):
    def __init__(self):
        super().__init__('wall_follower')
        # Declare parameters for tuning the behavior
        self.declare_parameter('desired_distance', 0.45)
        self.declare_parameter('linear_speed', 1.6)
        self.declare_parameter('kp', 2.0) # for the angular magnimizations
        self.declare_parameter('max_angular_speed', 1.3)
        self.declare_parameter('window_size', 10)  # number of indices on each side to consider
        self.declare_parameter('safety_distance', 0.3)  # for front obstacle detection

        self.desired_distance = self.get_parameter('desired_distance').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.kp = self.get_parameter('kp').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.window_size = self.get_parameter('window_size').value
        self.safety_distance = self.get_parameter('safety_distance').value
                # for plotting 
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.positions = []  # To store (x, y) positions
        self.world_size = 9  # Define the world size

        # Subscribers & Publishers
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)



        

    def odom_callback(self, msg):
        q = msg.pose.pose.orientation
        # Compute yaw from quaternion (only yaw needed for 2D navigation)
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.positions.append((x, y))
    
    def scan_callback(self, msg: LaserScan):
        # Calculate the index corresponding to the right side (approx. 270° or 4.71 rad)
        angle_min = msg.angle_min
        angle_inc = msg.angle_increment
        num_readings = len(msg.ranges)
        
        target_angle = 4.71  # roughly 270° in radians
        target_index = int((target_angle - angle_min) / angle_inc)
        
        # Define the window indices around the target index
        start_index = max(0, target_index - self.window_size)
        end_index = min(num_readings, target_index + self.window_size + 1)
        valid_readings = [
            msg.ranges[i] for i in range(start_index, end_index)
            if not np.isinf(msg.ranges[i]) and not np.isnan(msg.ranges[i])
        ]
        
        if valid_readings:
            # Compute error for each valid reading: desired_distance - actual reading.
            errors = [self.desired_distance - r for r in valid_readings]
            avg_error = np.mean(errors)
            mean_distance = np.mean(valid_readings)
        else:
            self.get_logger().warn("No valid LiDAR readings on right side. Stopping!")
            return

        # Proportional control for angular velocity based on the average error
        angular_z = self.kp * avg_error
        angular_z = max(min(angular_z, self.max_angular_speed), -self.max_angular_speed)
        
        # Adjust forward speed based on the magnitude of the average error
        adjusted_linear_speed = self.linear_speed * max(0.5, 1 - abs(avg_error))
        
        # Check for obstacles directly in front of the robot
        front_sector_deg = 10  # degrees on each side of the forward direction
        front_sector_rad = np.deg2rad(front_sector_deg)
        front_index_range = int(front_sector_rad / angle_inc)
        center_index = int((0 - angle_min) / angle_inc)
        
        front_readings = [
            msg.ranges[i] for i in range(max(0, center_index - front_index_range),
                                         min(num_readings, center_index + front_index_range + 1))
            if not np.isinf(msg.ranges[i]) and not np.isnan(msg.ranges[i])
        ]
        if front_readings and min(front_readings) < self.safety_distance:
            self.get_logger().warn("Obstacle detected in front! Stopping forward motion.")
            adjusted_linear_speed = 0.0

        self.get_logger().info(
            f"Mean distance: {mean_distance:.2f}, Avg error: {avg_error:.2f}, "
            f"angular_z: {angular_z:.2f}, linear: {adjusted_linear_speed:.2f}"
        )
        
        # Publish the computed Twist message
        twist = Twist()
        twist.linear.x = adjusted_linear_speed
        twist.angular.z = angular_z
        self.cmd_pub.publish(twist)
    
    def publish_stop(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)

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

        plt.savefig("/home/zaynap/project_ws/path_plotWF.png")
        # plt.show()
        
        # Plot smoothness and deviation
        plt.figure(figsize=(10, 5))
        plt.plot(range(len(smoothness)), smoothness, label="Smoothness")
        plt.plot(range(len(deviations)), deviations, label="Deviation")
        plt.xlabel("Point Index")
        plt.ylabel("Value")
        plt.title("Smoothness and Standard Deviation along the Path")
        plt.legend()
        # plt.gca().invert_yaxis()  # Match world view perspective

        plt.savefig("/home/zaynap/project_ws/smoothness_and_deviationWF.png")
        plt.show()
    

def main(args=None):
    rclpy.init(args=args)
    node = WallFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # node.publish_stop() 
        node.save_path_plot()  # Save the path when stopping
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
