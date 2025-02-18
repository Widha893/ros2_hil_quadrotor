import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Pose
import csv
import os
import time

class dataAnalyzer(Node):
    def __init__(self):
        super().__init__('data_analyzer_node')

        self.time_data = []
        self.alt_data = []

        self.csv_filename = '/home/widha893/data_log_alt.csv'
        self.fieldnames = ['timestamp', 'altitude']

        if not os.path.exists(self.csv_filename):
            with open(self.csv_filename, mode='w', newline='') as file:
                writer = csv.DictWriter(file, fieldnames=self.fieldnames)
                writer.writeheader()

        self.alt = 0.0

        self.alt_topic = '/simple_drone/gt_pose'
        self.alt_subscriber = self.create_subscription(Pose, self.alt_topic, self.alt_callback, 10)

    def alt_callback(self,msg:Pose):
        current_time = self.get_clock().now().to_msg().sec + self.get_clock().now().to_msg().nanosec * 1e-9
        self.alt = msg.position.z
        self.get_logger().info(f"altitude : {self.alt:.2f}")
        self.time_data.append(current_time)
        self.alt_data.append(self.alt)
        self.save_to_csv()

    def save_to_csv(self):
        timestamp = int(time.time() * 1e3)  # timestamp in microseconds
        # Append data to CSV
        with open(self.csv_filename, mode='a', newline='') as file:
            writer = csv.DictWriter(file, fieldnames=self.fieldnames)
            writer.writerow({
                'timestamp': timestamp,
                'altitude': self.alt,
            })

def main(args=None):
    rclpy.init(args=args)
    analyzer_node = dataAnalyzer()
    try:
        rclpy.spin(analyzer_node)
    except KeyboardInterrupt:
        analyzer_node.get_logger().info("Shutting down analyzer node...")
    finally:
        analyzer_node.destroy_node()
        rclpy.shutdown()