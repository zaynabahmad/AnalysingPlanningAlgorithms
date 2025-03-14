import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np
import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry
import pandas as pd 


class DualWallFollower(Node):
    def __init__(self):
        super().__init__('dual_wall_follower')
        # Declare parameters for tuning the behavior
        self.declare_parameter('desired_distance', 0.8)
        self.declare_parameter('linear_speed', 2.0)
        self.declare_parameter('kp', 4.0)
        self.declare_parameter('max_angular_speed', 0.8)
        self.declare_parameter('window_size', 20)         # number of indices on each side to consider for each wall
        self.declare_parameter('safety_distance', 0.3)      # for front obstacle detection
        
        # New parameters for turn detection
        self.declare_parameter('turn_detection_factor', 1.5)  # factor beyond desired_distance to consider a wall "missing"
        self.declare_parameter('extra_turn_gain', 0.5)          # additional gain to help in turns

        self.desired_distance = self.get_parameter('desired_distance').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.kp = self.get_parameter('kp').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.window_size = self.get_parameter('window_size').value
        self.safety_distance = self.get_parameter('safety_distance').value
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.positions = []  # To store (x, y) positions
        self.world_size = 9  # Define the world size
        
        self.turn_detection_factor = self.get_parameter('turn_detection_factor').value
        self.extra_turn_gain = self.get_parameter('extra_turn_gain').value

        # Subscribers & Publishers
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

    def odom_callback(self, msg):
        """Extracts position and yaw from odometry and stores it."""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.positions.append((x, y))
        
        # Limit stored positions to avoid excessive memory use
        if len(self.positions) > 1000:
            self.positions.pop(0)

    
    def scan_callback(self, msg: LaserScan):
        angle_min = msg.angle_min
        angle_inc = msg.angle_increment
        num_readings = len(msg.ranges)

        # For a 360° LiDAR:
        # Left wall target angle ~ 90° (pi/2 radians)
        # Right wall target angle ~ 270° (3*pi/2 radians)
        left_target_angle = np.pi / 2       # ~1.57 rad
        right_target_angle = 3 * np.pi / 2    # ~4.71 rad

        left_target_index = int((left_target_angle - angle_min) / angle_inc)
        right_target_index = int((right_target_angle - angle_min) / angle_inc)

        # Get window indices and valid readings for left wall
        left_start = max(0, left_target_index - self.window_size)
        left_end = min(num_readings, left_target_index + self.window_size + 1)
        left_valid = [msg.ranges[i] for i in range(left_start, left_end)
                      if not np.isinf(msg.ranges[i]) and not np.isnan(msg.ranges[i])]
        if left_valid:
            measured_left_index = np.argmin(left_valid)
            measured_left = left_valid[measured_left_index]
        else:
            self.get_logger().warn("No valid LiDAR readings on left side. Stopping!")
            return

        # Get window indices and valid readings for right wall
        right_start = max(0, right_target_index - self.window_size)
        right_end = min(num_readings, right_target_index + self.window_size + 1)
        right_valid = [msg.ranges[i] for i in range(right_start, right_end)
                       if not np.isinf(msg.ranges[i]) and not np.isnan(msg.ranges[i])]
        if right_valid:
            measured_right_index = np.argmin(right_valid)
            measured_right = right_valid[measured_right_index]
        else:
            self.get_logger().warn("No valid LiDAR readings on right side. Stopping!")
            return

        # Compute centering error.
        centering_error = measured_left - measured_right

        # Proportional control for angular velocity based on the centering error.
        angular_z = self.kp * centering_error

        # --- Added Logic for 45-Degree Turn Detection ---
        # If one wall is significantly farther than the desired distance,
        # assume a turn is approaching and add extra angular correction.
        if measured_left > self.desired_distance * self.turn_detection_factor:
            self.get_logger().info("Left turn detected: extra correction applied.")
            angular_z += self.extra_turn_gain * (measured_left - self.desired_distance)
        elif measured_right > self.desired_distance * self.turn_detection_factor:
            self.get_logger().info("Right turn detected: extra correction applied.")
            angular_z -= self.extra_turn_gain * (measured_right - self.desired_distance)
        # ---------------------------------------------------

        # Saturate angular_z to max_angular_speed
        angular_z = max(min(angular_z, self.max_angular_speed), -self.max_angular_speed)

        # Adjust forward speed based on the centering error.
        adjusted_linear_speed = self.linear_speed * max(0.5, 1 - abs(centering_error))

        # Check for obstacles directly in front of the robot.
        front_sector_deg = 10  # degrees on each side of the forward direction.
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
            f"Left: {measured_left:.2f}, Right: {measured_right:.2f}, CenterErr: {centering_error:.2f}, "
            f"angular_z: {angular_z:.2f}, linear: {adjusted_linear_speed:.2f}"
        )

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
        best_path_df = pd.read_csv("/home/zaynap/project_ws/trajectory.csv")

        if not self.positions:
            self.get_logger().warn("No positions found, not saving.")
            return

        # Extract actual path
        x_vals, y_vals = zip(*self.positions)
        
        # Extract best path from CSV
        best_x = best_path_df["x"].tolist()
        best_y = best_path_df["y"].tolist()

        
        actual = np.array(self.positions)
        optimal = np.array(list(zip(best_x, best_y)))


# Ensure both have the same length
        min_length = min(len(actual), len(optimal))
        actual = actual[:min_length]
        optimal = optimal[:min_length]

        rms_error = np.sqrt(np.mean((actual - optimal) ** 2))

        # Plot actual vs best path
        plt.figure(figsize=(6, 6))
        plt.plot(y_vals, x_vals, marker='o', markersize=2, linestyle='-', color='blue', label="Actual Path")
        plt.plot(best_y, best_x, marker='s', markersize=2, linestyle='--', color='red', label="Best Path")
        plt.xlim(22,-8)
        plt.ylim(30 , -3 )
        plt.xlabel("Y Axis")
        plt.ylabel("X Axis (Vertical)")
        plt.title(f"Robot Path (RMS Error: {rms_error:.3f})")

        plt.legend()
        plt.grid()
        plt.gca().invert_yaxis()  # Match world view perspective

        # Save plot
        plt.savefig("/home/zaynap/project_ws/trajectorycenterline_comparison.png")
        plt.show()

        self.get_logger().info(f"Saved trajectory plot with RMS error: {rms_error:.3f}")
        

def main(args=None):
    rclpy.init(args=args)
    node = DualWallFollower()
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
