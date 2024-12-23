import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, Range
import numpy as np
from std_msgs.msg import Bool, Float64MultiArray
import math
from hitl import messages_pb2
from serial import Serial, SerialException
import time
import csv
import os
import serial

STX = b'\xFE'
MAX_MESSAGE_SIZE = 1024  # Maximum buffer size, adjust if needed

class Communication(Node):
    def __init__(self,port = '/dev/ttyACM0', baudrate = 9600):
        super().__init__('communication')

        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.angular_vel_x = 0.0
        self.angular_vel_y = 0.0
        self.angular_vel_z = 0.0
        self.altitude = 0.0

        try:
            self._serial = Serial(port, baudrate, timeout=1)
        except SerialException as e:
            self.get_logger().error(f"Serial connection error: {e}")
            raise

        self.get_logger().info("Communication node has started...")

        # Define topics
        self.imu_topic = '/simple_drone/imu/out'
        self.alt_topic = 'simple_drone/sonar/out'

        # Obtained by running hitl_control.py
        self.gain_alt = 10.000
        self.gain_vz = 5.626
        self.gain_roll = 10.000
        self.gain_p = 5.000
        self.gain_pitch = 10.000
        self.gain_q = 5.000
        self.gain_yaw = 2.049
        self.gain_r = 1.000

        # Define Quality of Service
        qos = 10

        self.imu_updated = False
        self.alt_updated = False

        # Create publishers
        # self.pubHITL = self.create_publisher(Bool, '~/hitl', 1024)

        # Create subscribers
        self.imu_subscriber = self.create_subscription(
            Imu,self.imu_topic,self.imu_callback,qos
        )
        # self.alt_subscriber = self.create_subscription(
        #     Range,self.alt_topic,self.alt_callback,qos
        # )

        # CSV file setup
        # self.csv_filename = '/home/widha893/DataLogger/data_log.csv'
        # self.fieldnames = ['timestamp', 'roll', 'pitch', 'yaw', 'altitude', 'roll_setpoint', 'pitch_setpoint', 'yaw_setpoint']
        
        # if not os.path.exists(self.csv_filename):
        #     with open(self.csv_filename, mode='w', newline='') as file:
        #         writer = csv.DictWriter(file, fieldnames=self.fieldnames)
        #         writer.writeheader()

        # self.mHITL(True)

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
        if self.imu_updated:
            # self.save_to_csv()
            self.write(self.create_message())
            self.get_logger().info("Message sent!")
            self.reset_flags()

    def alt_callback(self, msg: Range):
        self.altitude = msg.range
        self.alt_updated = True
        if self.imu_updated and self.alt_updated:
            # self.save_to_csv()
            self.write(self.create_message())
            self.get_logger().info("Message sent!")
            self.reset_flags()

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

    def reset_flags(self):
        self.imu_updated = False
        self.alt_updated = False

    # def save_to_csv(self):
    #     timestamp = int(time.time() * 1e6)  # timestamp in microseconds
    #     roll_setpoint = 0.0
    #     pitch_setpoint = 0.0
    #     yaw_setpoint = 0.0

    #     # Append data to CSV
    #     with open(self.csv_filename, mode='a', newline='') as file:
    #         writer = csv.DictWriter(file, fieldnames=self.fieldnames)
    #         writer.writerow({
    #             'timestamp': timestamp,
    #             'roll': self.roll,
    #             'pitch': self.pitch,
    #             'yaw': self.yaw,
    #             'altitude': self.altitude,
    #             'roll_setpoint': roll_setpoint,
    #             'pitch_setpoint': pitch_setpoint,
    #             'yaw_setpoint': yaw_setpoint
    #         })

    def create_message(self):
        _msg = messages_pb2.msg()
        _msg.sensors.angular_vel_x = self.angular_vel_x
        _msg.sensors.angular_vel_y = self.angular_vel_y
        _msg.sensors.angular_vel_z = self.angular_vel_z
        _msg.sensors.roll = self.roll
        _msg.sensors.pitch = self.pitch
        _msg.sensors.yaw = self.yaw
        _msg.sensors.altitude = 0.0

        return _msg
    
    def write(self, message):
        buffer = message.SerializeToString()
        message_length = len(buffer)

        self._serial.write(STX)
        self._serial.write((message_length >> 8).to_bytes(1, 'big'))
        self._serial.write((message_length & 0xFF).to_bytes(1, 'big'))
        self._serial.write(buffer)

        checksum = 0x00
        for i in range(message_length):
            checksum ^= buffer[i]
        cb = checksum.to_bytes(1, 'big')
        self._serial.write(cb)
        self._serial.flush()

    # def save_graph(self):
    #     # Plot the roll, pitch, and yaw data with 0-degree setpoints
    #     fig, ax = plt.subplots()
    #     ax.plot(self.time_data, np.zeros_like(self.time_data), 'k--', label='Setpoint (0 degrees)')
    #     ax.plot(self.time_data, self.roll_data, label='Roll', color='r')
    #     ax.plot(self.time_data, self.pitch_data, label='Pitch', color='g')
    #     ax.plot(self.time_data, self.yaw_data, label='Yaw', color='b')

    #     ax.set_xlabel('Time (seconds)')
    #     ax.set_ylabel('Angle (degrees)')
    #     ax.set_title('Roll, Pitch, and Yaw over Time')
    #     ax.legend()

    #     # Save the plot as an image
    #     os.makedirs('graphs', exist_ok=True)
    #     plot_filename = f'/home/widha893/graphs/roll_pitch_yaw_{time.strftime("%Y%m%d_%H%M%S")}.png'
    #     plt.savefig(plot_filename)
    #     self.get_logger().info(f"Graph saved to {plot_filename}")
    #     plt.close(fig)

def main(args=None):
    rclpy.init(args=args)
    communication = Communication()
    try:
        rclpy.spin(communication)
    except SerialException as e:
        communication.get_logger().error(f"Serial connection error: {e}")
    except KeyboardInterrupt:
        communication.get_logger().info("Shutting down due to KeyboardInterrupt...")
        # communication.save_graph()
        communication.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()