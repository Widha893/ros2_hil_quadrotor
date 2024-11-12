import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import numpy as np
import csv
import os
from datetime import datetime

class SensorDataLogger(Node):
    def __init__(self):
        super().__init__('data_logger')

        self.sensor_data = []
        
        # Define the IMU topic
        self.imu_topic = '/simple_drone/imu/out'  # Replace with actual IMU topic name
        qos = 10  # Quality of Service (can be modified as needed)
        
        if self.imu_topic:
            # Create the IMU subscriber
            self.imu_subscriber = self.create_subscription(
                Imu,
                self.imu_topic,
                self.imu_callback,
                qos
            )
            self.get_logger().info(f"Subscribed to IMU topic: {self.imu_topic}")
        else:
            self.get_logger().error("No IMU topic defined!")

    def _imu_callback(self, msg: Imu):
        # Extract quaternion and convert to Euler angles
        quat = msg.orientation
        roll, pitch, yaw = self.quaternion_to_euler(quat)

        # Log Euler angles
        self.get_logger().info(f"Euler Angles [Degrees]: Roll={np.degrees(roll):.2f}, Pitch={np.degrees(pitch):.2f}, Yaw={np.degrees(yaw):.2f}")

    def imu_callback(self, msg: Imu):
        # Extract quaternion and convert to Euler angles
        quat = msg.orientation
        roll, pitch, yaw = self.quaternion_to_euler(quat)

        # Log Euler angles
        self.get_logger().info(f"Euler Angles [Degrees]: Roll={np.degrees(roll):.2f}, Pitch={np.degrees(pitch):.2f}, Yaw={np.degrees(yaw):.2f}")

        timestamp_sec = msg.header.stamp.sec
        timestamp_nanosec = msg.header.stamp.nanosec
        self.sensor_data.append([timestamp_sec, timestamp_nanosec, np.degrees(roll), np.degrees(pitch), np.degrees(yaw)])

    
    def quaternion_to_euler(self, quat):
        """Convert a quaternion to Euler angles (roll, pitch, yaw)"""
        q = [quat.w, quat.x, quat.y, quat.z]
        
        sinr_cosp = 2 * (q[0] * q[1] + q[2] * q[3])
        cosr_cosp = 1 - 2 * (q[1]**2 + q[2]**2)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (q[0] * q[2] - q[3] * q[1])
        pitch = np.arcsin(np.clip(sinp, -1.0, 1.0))

        siny_cosp = 2 * (q[0] * q[3] + q[1] * q[2])
        cosy_cosp = 1 - 2 * (q[2]**2 + q[3]**2)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def save_data(self):
        """Saves collected sensor data to a CSV file."""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        file_path = os.path.expanduser(f'~/DataLogger/data_sensors_{timestamp}.csv')
        # file_path = '/DataLogger/data_sensors.csv'
        os.makedirs(os.path.dirname(file_path), exist_ok=True)  # Ensure directory exists
        
        self.get_logger().info(f"Saving data to {file_path}")
        try:
            with open(file_path, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    'timestamp_sec', 'timestamp_nanosec',
                    'roll', 'pitch', 'yaw'
                ])
                writer.writerows(self.sensor_data)
            self.get_logger().info("Data successfully saved to CSV.")
        except Exception as e:
            self.get_logger().error(f"Failed to save data: {e}")

def main(args=None):
    rclpy.init(args=args)
    sensor_data_logger = SensorDataLogger()

    try:
        rclpy.spin(sensor_data_logger)
    except KeyboardInterrupt:
        sensor_data_logger.get_logger().info("Shutting down due to KeyboardInterrupt...")
    finally:
        # Save data on shutdown
        sensor_data_logger.save_data()
        sensor_data_logger.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()