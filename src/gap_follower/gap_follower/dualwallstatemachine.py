import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np

class DualWallFollower(Node):
    def __init__(self):
        super().__init__('dual_wall_follower')
        # In a straight corridor 1.0 m wide, the ideal distance from each wall is 0.5 m.
        self.declare_parameter('desired_distance', 0.5)
        self.declare_parameter('linear_speed', 2.0)
        self.declare_parameter('kp', 4.0)
        self.declare_parameter('max_angular_speed', 1.0)
        self.declare_parameter('window_size', 20)         # number of indices to consider around target angle for each wall
        self.declare_parameter('safety_distance', 0.4)      # for front obstacle detection

        # Parameters for detecting a turn (when the wall is much farther than in a straight corridor)
        self.declare_parameter('turn_detection_factor', 1.2)  # e.g., 0.5*1.2 = 0.75 m threshold
        self.declare_parameter('extra_turn_gain', 0.5)          # extra gain applied during a turn

        self.desired_distance = self.get_parameter('desired_distance').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.kp = self.get_parameter('kp').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.window_size = self.get_parameter('window_size').value
        self.safety_distance = self.get_parameter('safety_distance').value
        self.turn_detection_factor = self.get_parameter('turn_detection_factor').value
        self.extra_turn_gain = self.get_parameter('extra_turn_gain').value

        # Initialize state machine; states: "NORMAL", "TURN_LEFT", "TURN_RIGHT"
        self.state = "NORMAL"

        # Subscribers & Publishers
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def scan_callback(self, msg: LaserScan):
        angle_min = msg.angle_min
        angle_inc = msg.angle_increment
        num_readings = len(msg.ranges)

        # Target angles:
        # Left wall ~ 90° (pi/2) and Right wall ~ 270° (3*pi/2)
        left_target_angle = np.pi / 2       # ~1.57 rad
        right_target_angle = 3 * np.pi / 2    # ~4.71 rad

        left_target_index = int((left_target_angle - angle_min) / angle_inc)
        right_target_index = int((right_target_angle - angle_min) / angle_inc)

        # --- Extract valid ranges for left wall ---
        left_start = max(0, left_target_index - self.window_size)
        left_end = min(num_readings, left_target_index + self.window_size + 1)
        valid_left = [msg.ranges[i] for i in range(left_start, left_end)
                      if not np.isinf(msg.ranges[i]) and not np.isnan(msg.ranges[i])]
        if valid_left:
            # In a straight corridor, the wall should be about desired_distance (0.5 m).
            # We use the minimum valid reading (closest wall) as measured distance.
            measured_left = min(valid_left)
        else:
            self.get_logger().warn("No valid LiDAR readings on left side. Stopping!")
            return

        # --- Extract valid ranges for right wall ---
        right_start = max(0, right_target_index - self.window_size)
        right_end = min(num_readings, right_target_index + self.window_size + 1)
        valid_right = [msg.ranges[i] for i in range(right_start, right_end)
                       if not np.isinf(msg.ranges[i]) and not np.isnan(msg.ranges[i])]
        if valid_right:
            measured_right = min(valid_right)
        else:
            self.get_logger().warn("No valid LiDAR readings on right side. Stopping!")
            return

        # --- Determine if a turn is occurring ---
        # In a straight corridor, each wall should be about desired_distance.
        # If a wall is sensed at a distance significantly greater than desired_distance,
        # then we assume a turn (the corridor widens at the turn).
        previous_state = self.state
        if measured_left > self.desired_distance * self.turn_detection_factor:
            self.state = "TURN_LEFT"
        elif measured_right > self.desired_distance * self.turn_detection_factor:
            self.state = "TURN_RIGHT"
        else:
            self.state = "NORMAL"

        if self.state != previous_state:
            self.get_logger().info(f"State changed: {previous_state} -> {self.state}")

        # --- Compute centering error ---
        # In a straight section, if the robot is centered, measured_left should equal measured_right.
        # The centering error is the difference between the two.
        centering_error = measured_left - measured_right

        # --- Compute control based on state ---
        if self.state == "NORMAL":
            # In a straight corridor, use proportional correction based on centering error.
            angular_z = self.kp * centering_error
            adjusted_linear_speed = self.linear_speed * max(0.5, 1 - abs(centering_error))
        elif self.state == "TURN_LEFT":
            # When turning left, the left wall is farther than expected.
            angular_z = self.kp * centering_error + self.extra_turn_gain * (measured_left - self.desired_distance)
            adjusted_linear_speed = self.linear_speed * 0.5  # reduce speed for safer turning
        elif self.state == "TURN_RIGHT":
            # When turning right, the right wall is farther than expected.
            angular_z = self.kp * centering_error - self.extra_turn_gain * (measured_right - self.desired_distance)
            adjusted_linear_speed = self.linear_speed * 0.5
        else:
            angular_z = self.kp * centering_error
            adjusted_linear_speed = self.linear_speed

        # Saturate angular velocity
        angular_z = max(min(angular_z, self.max_angular_speed), -self.max_angular_speed)

        # --- Check for obstacles in front ---
        front_sector_deg = 10  # degrees on each side of forward direction
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

        # Log values for debugging
        self.get_logger().info(
            f"State: {self.state} | Left: {measured_left:.2f}, Right: {measured_right:.2f}, "
            f"CenterErr: {centering_error:.2f}, angular_z: {angular_z:.2f}, "
            f"linear: {adjusted_linear_speed:.2f}"
        )

        # Publish command velocities
        twist = Twist()
        twist.linear.x = adjusted_linear_speed
        twist.angular.z = angular_z
        self.cmd_pub.publish(twist)
    
    def publish_stop(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = DualWallFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_stop()  # Ensure the robot stops before shutdown.
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
