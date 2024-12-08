import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Pose
import numpy as np
import math
import csv
import os
from datetime import datetime
import time

class SensorDataLogger(Node):
    def __init__(self):
        super().__init__('data_logger')

        self.sensor_data = []
        
        # Define the IMU topic
        self.imu_topic = '/simple_drone/imu/out'  # Replace with actual IMU topic name
        self.altitude_topic = 'simple_drone/gt_pose'
        qos = 10  # Quality of Service (can be modified as needed)

        self.imu_status = False
        self.alt_status = False
        
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

        if self.altitude_topic:
            # Create the alt subscriber
            self.altitude_subscriber = self.create_subscription(
                Pose,
                self.altitude_topic,
                self.altitude_callback,
                qos
            )
            self.get_logger().info(f"Subscribed to IMU topic: {self.imu_topic}")
        else:
            self.get_logger().error("No IMU topic defined!")

        self.csv_filename = '/home/widha893/DataLogger/data_log_new.csv'
        self.fieldnames = ['timestamp', 'roll', 'pitch', 'yaw', 'altitude', 'roll_setpoint', 'pitch_setpoint', 'yaw_setpoint', 'alt_setpoint']
        
        if not os.path.exists(self.csv_filename):
            with open(self.csv_filename, mode='w', newline='') as file:
                writer = csv.DictWriter(file, fieldnames=self.fieldnames)
                writer.writeheader()

    def imu_callback(self, msg: Imu):
        # Extract quaternion and convert to Euler angles
        quat = msg.orientation
        self.roll = np.degrees(self.quaternion_to_roll(quat.w, quat.x, quat.y, quat.z))
        self.pitch = np.degrees(self.quaternion_to_pitch(quat.w, quat.x, quat.y, quat.z))
        self.yaw = np.degrees(self.quaternion_to_yaw(quat.w, quat.x, quat.y, quat.z))
        self.angular_vel_x = msg.angular_velocity.x
        self.angular_vel_y = msg.angular_velocity.y
        self.angular_vel_z = msg.angular_velocity.z
        self.imu_updated = True
    
    def quaternion_to_roll(self, q_w, q_x, q_y, q_z):
        # Roll (x-axis rotation)
        roll = math.atan2(2 * (q_w * q_x + q_y * q_z), 1 - 2 * (q_x**2 + q_y**2))
        return roll

    def quaternion_to_pitch(self, q_w, q_x, q_y, q_z):
        pitch = math.asin(2 * (q_w * q_y - q_z * q_x))
        return pitch

    def quaternion_to_yaw(self, q_w, q_x, q_y, q_z):
        yaw = math.atan2(2 * (q_w * q_z + q_x * q_y), 1 - 2 * (q_y**2 + q_z**2))
        return yaw
    
    def altitude_callback(self, msg: Pose):
        self.altitude = msg.position.z
        self.alt_updated = True
        if self.imu_updated and self.alt_updated:
            self.get_logger().info(f"Altitude = {self.altitude:.2f}, Euler Angles [Degrees]: Roll={self.roll:.2f}, Pitch={self.pitch:.2f}, Yaw={self.yaw:.2f}")
            self.save_to_csv()
            self.reset_flags()

    def save_to_csv(self):
        timestamp = int(time.time() * 1e3)  # timestamp in milliseconds
        roll_setpoint = 0.0
        pitch_setpoint = 0.0
        yaw_setpoint = 0.0
        alt_setpoint = 0.8

        # Append data to CSV
        with open(self.csv_filename, mode='a', newline='') as file:
            writer = csv.DictWriter(file, fieldnames=self.fieldnames)
            writer.writerow({
                'timestamp': timestamp,
                'roll': self.roll,
                'pitch': self.pitch,
                'yaw': self.yaw,
                'altitude': 0.0,
                'roll_setpoint': roll_setpoint,
                'pitch_setpoint': pitch_setpoint,
                'yaw_setpoint': yaw_setpoint,
                'alt_setpoint': alt_setpoint
            })

    def reset_flags(self):
        self.imu_status = False
        self.alt_status = False

def main(args=None):
    rclpy.init(args=args)
    sensor_data_logger = SensorDataLogger()

    try:
        rclpy.spin(sensor_data_logger)
    except KeyboardInterrupt:
        sensor_data_logger.get_logger().info("Shutting down due to KeyboardInterrupt...")
    finally:
        # Save data on shutdown
        sensor_data_logger.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()