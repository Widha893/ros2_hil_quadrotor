import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, Range
from std_msgs.msg import Float64MultiArray, Bool
import numpy as np
import math
import time
from serial import Serial, SerialException
from hitl import messages_pb2


STX = b'\xFE'  # Start byte for protocol
MAX_MESSAGE_SIZE = 1024  # Maximum buffer size


class HITLNode(Node):
    def __init__(self, port='/dev/ttyACM0', baudrate=9600):
        super().__init__('hitl_node')

        # Initialize sensor data
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

        self.get_logger().info("HITL node has started...")

        # Topics for sensor data
        self.imu_topic = '/simple_drone/imu/out'
        self.alt_topic = '/simple_drone/sonar/out'

        # Initialize state flags
        self.imu_updated = False
        self.alt_updated = False

        # Define control signal gains (adjust as needed)
        self.gain_alt = 10.000
        self.gain_vz = 5.626
        self.gain_roll = 10.000
        self.gain_p = 5.000
        self.gain_pitch = 10.000
        self.gain_q = 5.000
        self.gain_yaw = 2.049
        self.gain_r = 1.000

        # Create publishers
        self.pub_control_signals = self.create_publisher(Float64MultiArray, '~/drone_control_signals', 10)
        self.pub_status = self.create_publisher(Bool, '~/hitl_status', 10)

        # Create subscribers
        self.imu_subscriber = self.create_subscription(Imu, self.imu_topic, self.imu_callback, 10)
        # self.alt_subscriber = self.create_subscription(Range, self.alt_topic, self.alt_callback, 10)

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
            self.send_to_micon()
            self.reset_flags()

    # def alt_callback(self, msg: Range):
    #     self.altitude = msg.range
    #     self.alt_updated = True

    #     if self.imu_updated and self.alt_updated:
    #         self.send_to_micon()
    #         self.reset_flags()

    def quaternion_to_roll(self, q_w, q_x, q_y, q_z):
        return math.atan2(2 * (q_w * q_x + q_y * q_z), 1 - 2 * (q_x**2 + q_y**2))

    def quaternion_to_pitch(self, q_w, q_x, q_y, q_z):
        return math.asin(2 * (q_w * q_y - q_z * q_x))

    def quaternion_to_yaw(self, q_w, q_x, q_y, q_z):
        return math.atan2(2 * (q_w * q_z + q_x * q_y), 1 - 2 * (q_y**2 + q_z**2))

    def send_to_micon(self):
        # Create message to send to micon
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
        _msg.sensors.altitude = 0.0

        try:
            # Send message to micon
            self._serial.write(STX + _msg.SerializeToString())
            # self.get_logger().info("Sensor data sent to microcontroller.")

            # Publish HITL status as True
            self.publish_hitl_status(True)

            self.receive_from_micon()  # Receive control signals back
        except Exception as e:
            self.get_logger().error(f"Error sending data to micon: {e}")
            self.publish_hitl_status(False)  # Publish HITL status as False

    def receive_from_micon(self):
        try:
            if self._serial.in_waiting > 0:
                data = self._serial.read(self._serial.in_waiting)
                if data.startswith(STX):  # Check for valid message
                    control_msg = messages_pb2.msg()
                    control_msg.ParseFromString(data[1:])  # Parse the response from micon
                    self.publish_control_signals(control_msg)
        except Exception as e:
            self.get_logger().error(f"Error receiving data from micon: {e}")
            self.publish_hitl_status(False)  # Publish HITL status as False

    def publish_control_signals(self, control_msg):
        # Publish control signals to control the drone
        control_signal_msg = Float64MultiArray()
        control_signal_msg.data = [control_msg.controls.total_force, control_msg.controls.torque_x,
                                   control_msg.controls.torque_y, control_msg.controls.torque_z]
        self.pub_control_signals.publish(control_signal_msg)
        self.get_logger().info(f"Published: {control_signal_msg.data}")

    def publish_hitl_status(self, status):
        # Publish the HITL status as a Boolean value
        hitl_status_msg = Bool()
        hitl_status_msg.data = status
        self.pub_status.publish(hitl_status_msg)
        # self.get_logger().info(f"HITL status published: {status}")

    def reset_flags(self):
        self.imu_updated = False
        self.alt_updated = False


def main(args=None):
    rclpy.init(args=args)
    node = HITLNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
