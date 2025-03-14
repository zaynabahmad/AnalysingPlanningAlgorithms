import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import csv

class OdomRecorder(Node):
    def __init__(self):
        super().__init__('odom_recorder')
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',  # تأكد من أن اسم التوبيك صحيح
            self.odom_callback,
            10)
        self.file = open("trajectory.csv", "w", newline='')
        self.writer = csv.writer(self.file)
        self.writer.writerow(["x", "y"])  # كتابة العناوين

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.writer.writerow([x, y])
        self.get_logger().info(f"Saved: x={x}, y={y}")

    def destroy_node(self):
        self.file.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    recorder = OdomRecorder()
    rclpy.spin(recorder)
    recorder.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
