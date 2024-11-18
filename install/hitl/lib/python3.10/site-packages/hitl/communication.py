import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, Range
import numpy as np
from std_msgs.msg import Bool
import math
import messages_pb2
from serial import Serial, SerialException

class Communication(Node):
    def __init__(self,port = '/dev/ttyACM0', baudrate = 9600):
        super().__init__('communication')

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
        self.euler = np.degrees(self.quaternion_to_euler(quat.w, quat.x, quat.y, quat.z))
        self.angular_vel_x = msg.angular_velocity.x
        self.angular_vel_y = msg.angular_velocity.y
        self.angular_vel_z = msg.angular_velocity.z
        self.imu_updated = True

    def alt_callback(self, msg: Range):
        self.altitude = msg.range
        self.alt_updated = True
        if self.imu_updated and self.alt_updated:
            self.write(self.create_message())
            self.reset_flags()

    def quaternion_to_euler(self, q_w, q_x, q_y, q_z):
        # Roll (x-axis rotation)
        roll = math.atan2(2 * (q_w * q_x + q_y * q_z), 1 - 2 * (q_x**2 + q_y**2))

        # Pitch (y-axis rotation)
        pitch = math.asin(2 * (q_w * q_y - q_z * q_x))

        # Yaw (z-axis rotation)
        yaw = math.atan2(2 * (q_w * q_z + q_x * q_y), 1 - 2 * (q_y**2 + q_z**2))
    
        return roll, pitch, yaw

    def reset_flags(self):
        self.imu_updated = False
        self.alt_updated = False

    def create_message(self):
        msg = messages_pb2.msg()
        msg.gains.alt = self.gain_alt
        msg.gains.vz = self.gain_vz
        msg.gains.roll = self.gain_roll
        msg.gains.p = self.gain_p
        msg.gains.pitch = self.gain_pitch
        msg.gains.q = self.gain_q
        msg.gains.yaw = self.gain_yaw
        msg.gains.r = self.gain_r
        msg.sensors.euler.roll = self.euler[0]
        msg.sensors.euler.pitch = self.euler[1]
        msg.sensors.euler.yaw = self.euler[2]
        msg.sensors.angular_vel.x = self.angular_vel_x
        msg.sensors.angular_vel.y = self.angular_vel_y
        msg.sensors.angular_vel.z = self.angular_vel_z
        msg.sensors.altitude = self.altitude

        return msg
    
    def write(self, message):
        STX = b'\x02'  # Define the Start of Transmission byte

        # Serialize the message
        buffer = message.SerializeToString()
        message_length = len(buffer)

        # Write the start byte (STX)
        self._serial.write(STX)

        # Write the message length in 2 bytes (big-endian)
        self._serial.write((message_length >> 8).to_bytes(1, 'big'))  # High byte
        self._serial.write((message_length & 0xFF).to_bytes(1, 'big'))  # Low byte

        # Write the serialized message
        self._serial.write(buffer)

        # Calculate checksum (XOR of all message bytes)
        checksum = 0x00
        for byte in buffer:
            checksum ^= byte

        # Write the checksum
        self._serial.write(checksum.to_bytes(1, 'big'))

        # Flush to ensure the data is sent immediately
        self._serial.flush()

        self.get_logger().info("Message sent with checksum: 0x{:02X}".format(checksum))


def main(args=None):
    rclpy.init(args=args)
    communication = Communication()
    try:
        rclpy.spin(communication)
    except KeyboardInterrupt:
        communication.get_logger().info("Shutting down due to KeyboardInterrupt...")
        communication.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()