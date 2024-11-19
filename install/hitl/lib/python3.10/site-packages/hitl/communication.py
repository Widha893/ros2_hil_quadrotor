import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, Range
import numpy as np
from std_msgs.msg import Bool
import math
from hitl import messages_pb2
from serial import Serial, SerialException
import time

STX = b'\xFE'

class Communication(Node):
    def __init__(self,port = '/dev/ttyACM0', baudrate = 115200):
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
        self.gain_vz = 5.526
        self.gain_roll = 5.000
        self.gain_p = 4.261
        self.gain_pitch = 5.477
        self.gain_q = 3.043
        self.gain_yaw = 3.000
        self.gain_r = 1.519

        # Define Quality of Service
        qos = 10

        self.imu_updated = False
        self.alt_updated = False

        # Create subscribers
        self.imu_subscriber = self.create_subscription(
            Imu,self.imu_topic,self.imu_callback,qos
        )
        self.alt_subscriber = self.create_subscription(
            Range,self.alt_topic,self.alt_callback,qos
        )

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

    def alt_callback(self, msg: Range):
        self.altitude = msg.range
        self.alt_updated = True
        if self.imu_updated and self.alt_updated:
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

    def create_message(self):
        _msg = messages_pb2.msg()
        _msg.gains.alt = self.gain_alt
        _msg.gains.vz = self.gain_vz
        _msg.gains.roll = self.gain_roll
        _msg.gains.p = self.gain_p
        _msg.gains.pitch = self.gain_pitch
        _msg.gains.q = self.gain_q
        _msg.gains.yaw = self.gain_yaw
        _msg.gains.r = self.gain_r
        _msg.sensors.angular_vel_x = self.angular_vel_x
        _msg.sensors.angular_vel_y = self.angular_vel_y
        _msg.sensors.angular_vel_z = self.angular_vel_z
        _msg.sensors.roll = self.roll
        _msg.sensors.pitch = self.pitch
        _msg.sensors.yaw = self.yaw
        _msg.sensors.altitude = self.altitude

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
        time.sleep(0.5)


def main(args=None):
    rclpy.init(args=args)
    communication = Communication()
    try:
        rclpy.spin(communication)
    except SerialException as e:
        communication.get_logger().error(f"Serial connection error: {e}")
    except KeyboardInterrupt:
        communication.get_logger().info("Shutting down due to KeyboardInterrupt...")
        communication.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()