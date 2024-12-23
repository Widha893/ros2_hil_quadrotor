import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Imu, Range
from std_msgs.msg import Float64MultiArray, Bool, Float64
import numpy as np
import math
import time
from serial import Serial, SerialException
from hitl import messages_pb2
import struct


STX = b'\xFE'  # Start byte for protocol
STX_receive = 0xFE  # Start byte for protocol
MAX_MESSAGE_SIZE = 1024  # Maximum buffer size


class HITLNode(Node):
    def __init__(self):
        super().__init__('hitl_node')

        self.port = '/dev/ttyACM0'
        self.baudrate = 6000000
        self.serial_port = None

        # Initialize sensor data
        # self.roll = 0.0
        # self.pitch = 0.0
        # self.yaw = 0.0
        self.orientation_w = 0.0
        self.orientation_x = 0.0
        self.orientation_y = 0.0
        self.orientation_z = 0.0
        self.altitude = 0.0
        self.roll_command = 0.0
        self.pitch_command = 0.0
        self.gyro_x = 0.0
        self.gyro_y = 0.0
        self.gyro_z = 0.0
        # self.prev_quat = None
        # self.prev_time = None
        # self.angular_velocity = [0.0, 0.0, 0.0]

        self.disturbance_interval = 10.0  # seconds
        self.disturbance_duration = 1.0  # seconds
        self.disturbance_applied = False
        self.last_disturbance_time = self.get_clock().now().seconds_nanoseconds()[0]

        try:
            self.serial_port = Serial(self.port, self.baudrate, timeout=1)
            self.serial_port.reset_input_buffer()  # Clear any initial garbage data in the buffer
            self.get_logger().info("Serial connection established.")

            # Send handshake 'R' to Teensy
            self.get_logger().info("Sending handshake...")
            self.serial_port.write(b'R')  # Send 'R' as a handshake signal
            self.serial_port.flush()
            # self.serial_port.reset_input_buffer()
            self.serial_port.reset_output_buffer()
            self.get_logger().info("Handshake sent to Teensy.")
        except SerialException as e:
            self.get_logger().error(f"Serial connection error: {e}")
            raise

        self.get_logger().info("HITL node has started...")

        # Topics for sensor data
        self.imu_topic = '/simple_drone/imu/out'
        self.alt_topic = '/simple_drone/sonar/out'

        # Define control signal gains (adjust as needed)
        # self.gain_alt = 10.000
        # self.gain_vz = 5.626
        # self.gain_roll = 10.000 # 20.00
        # self.gain_p = 5.000 # 10.228
        # self.gain_pitch = 10.000 # 20.00
        # self.gain_q = 5.000 # 10.228
        # self.gain_yaw = 2.071
        # self.gain_r = 1.099

        # Create publishers
        self.pub_control_signals = self.create_publisher(Float64MultiArray, '~/drone_control_signals', 10)
        self.pub_status = self.create_publisher(Bool, '~/hitl_status', 10)

        # Create subscribers

        self.imu_subscriber = self.create_subscription(Imu, self.imu_topic, self.imu_callback, 10)
        # self.alt_subscriber = self.create_subscription(Range, self.alt_topic, self.alt_callback, 10)

    def imu_callback(self, msg: Imu):
        self.orientation_w = msg.orientation.w
        self.orientation_x = msg.orientation.x
        self.orientation_y = msg.orientation.y
        self.orientation_z = msg.orientation.z
        self.gyro_x = msg.angular_velocity.x
        self.gyro_y = msg.angular_velocity.y
        self.gyro_z = msg.angular_velocity.z


    def roll_command_callback(self, msg: Float64):
        self.roll_command = msg.data

    def pitch_command_callback(self, msg: Float64):
        self.pitch_command = msg.data

    def alt_callback(self, msg: Range):
        try:
            self.altitude = msg.range
        except Exception as e:
            self.altitude = 0.0
        # self.get_logger().info(f"Altitude = {self.altitude:.2f}, Euler Angles [Degrees]: Roll={self.roll:.2f}, Pitch={self.pitch:.2f}, Yaw={self.yaw:.2f}")
        # self.disturbance_control()
        # self.send_to_micon()
        # self.poll_serial()
        # self.publish_hitl_status(True)

    def quaternion_to_roll(self, q_w, q_x, q_y, q_z):
        return math.atan2(2 * (q_w * q_x + q_y * q_z), 1 - 2 * (q_x**2 + q_y**2))
    
    def quaternion_to_pitch(self, q_w, q_x, q_y, q_z):
        return math.asin(2 * (q_w * q_y - q_z * q_x))
    
    def quaternion_to_yaw(self, q_w, q_x, q_y, q_z):
        return math.atan2(2 * (q_w * q_z + q_x * q_y), 1 - 2 * (q_y**2 + q_z**2))
    
    def quaternion_to_angular_velocity(self, q1, q0, dt):

        # Convert inputs to numpy arrays
        q1 = np.array(q1)
        q0 = np.array(q0)

        # Normalize quaternions
        q1 = q1 / np.linalg.norm(q1)
        q0 = q0 / np.linalg.norm(q0)

        # Compute quaternion difference: delta_q = q1 * inverse(q0)
        q0_inv = np.array([q0[0], -q0[1], -q0[2], -q0[3]])  # Quaternion inverse
        delta_q = self.quaternion_multiply(q1, q0_inv)

        # Extract angle of rotation from delta_q
        angle = 2 * np.arccos(np.clip(delta_q[0], -1.0, 1.0))  # Angle in radians
        if np.linalg.norm(delta_q[1:]) > 0:
            axis = delta_q[1:] / np.linalg.norm(delta_q[1:])  # Unit rotation axis
        else:
            axis = np.array([0.0, 0.0, 0.0])

        # Convert to angular velocity
        angular_velocity_rad_s = (axis * angle) / dt  # In radians/second
        angular_velocity_deg_s = np.degrees(angular_velocity_rad_s)  # Convert to degrees/second

        return angular_velocity_deg_s

    def quaternion_multiply(self, q1, q2):
        """
        Multiply two quaternions q1 and q2.

        Parameters:
        - q1, q2: Quaternions [w, x, y, z]

        Returns:
        - Resulting quaternion [w, x, y, z]
        """
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2

        w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
        x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
        y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
        z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2

        return np.array([w, x, y, z])

    def send_to_micon(self):
        # Create message to send to micon
        _msg = messages_pb2.msg()
        _msg.sensors.angular_vel_x = self.gyro_x
        _msg.sensors.angular_vel_y = self.gyro_y
        _msg.sensors.angular_vel_z = self.gyro_z
        _msg.sensors.orientation_w = self.orientation_w
        _msg.sensors.orientation_x = self.orientation_x
        _msg.sensors.orientation_y = self.orientation_y
        _msg.sensors.orientation_z = self.orientation_z
        _msg.sensors.altitude = 0.0
        _msg.command.roll = 0.0
        _msg.command.pitch = 0.0
        _msg.command.yaw = 0.0

        try:
            buffer = _msg.SerializeToString()
            message_length = len(buffer)
            self.serial_port.write(STX)
            self.serial_port.write((message_length >> 8).to_bytes(1, 'big'))
            self.serial_port.write((message_length & 0xFF).to_bytes(1, 'big'))
            self.serial_port.write(buffer)
            checksum = 0x00
            for i in range(message_length):
                checksum ^= buffer[i]
            cb = checksum.to_bytes(1, 'big')
            self.serial_port.write(cb)
            self.serial_port.flush()
        except Exception as e:
            self.get_logger().error(f"Error sending data to micon: {e}")
            self.publish_hitl_status(False)  # Publish HITL status as False

    def receive_message(self):
        """
        Receive and decode a Protobuf message from the serial port
        """
        try:
            # Wait for the start byte (STX)
            while True:
                self.get_logger().info("Waiting for STX")
                byte = self.serial_port.read(1)
                if not byte:
                    raise TimeoutError("Timeout waiting for STX")
                if byte[0] == STX_receive:
                    break

            # Read the message size (2 bytes, big-endian)
            size_bytes = self.serial_port.read(2)
            if len(size_bytes) != 2:
                raise ValueError("Incomplete message size")
            message_size = struct.unpack(">H", size_bytes)[0]

            if message_size > MAX_MESSAGE_SIZE:
                self.get_logger().error(f"Message size too large: {message_size}")
                return None

            # Read the message data
            message_data = self.serial_port.read(message_size)
            if len(message_data) != message_size:
                raise ValueError("Incomplete message data")

            # Calculate and validate checksum
            checksum = 0
            for byte in message_data:
                checksum ^= byte

            received_checksum = self.serial_port.read(1)
            if len(received_checksum) != 1:
                raise ValueError("Incomplete checksum")
            if checksum != received_checksum[0]:
                self.get_logger().error(
                    f"Checksum mismatch. Calculated: {checksum}, Received: {received_checksum[0]}"
                )
                return None

            # Parse the message using Protobuf
            hwil_message = messages_pb2.msg()
            hwil_message.ParseFromString(message_data)
            return hwil_message

        except Exception as e:
            self.get_logger().error(f"Error receiving message: {e}")
            return None
        
    def read(self):
        buffer = b''
        checksum = 0
        message = messages_pb2.msg()
        self.get_logger().info("trying to read")
        # self.get_logger().info(self.serial_port.in_waiting)
        while self.serial_port.in_waiting > 3:
            data = self.serial_port.read(1)
            if data == STX:
                message_size = self.serial_port.read(1)[0] << 8 | self.serial_port.read(1)[0]
                for i in range(message_size):
                    data = self.serial_port.read(1)
                    buffer += data
                    checksum ^= data[0]
                if checksum == self.serial_port.read(1)[0]:
                    message.ParseFromString(buffer)
                    self.get_logger().info("Message received")
                    return message
                else:
                    self.get_logger().error("Checksum mismatch")
                    return None
            else:
                self.get_logger().error("STX not found")
                
    def poll_serial(self):
        """
        Poll the serial port for data and publish it to the ROS 2 topic
        """
        try:
            self.get_logger().info("Polling serial port...")
            message = self.read()
            if message:
                # Debugging: Print received values
                self.get_logger().info(
                    f"Received data - Total Force: {message.controls.total_force:.2f}, "
                    f"Torque X: {message.controls.torque_x:.2f}, "
                    f"Torque Y: {message.controls.torque_y:.2f}, "
                    f"Torque Z: {message.controls.torque_z:.2f}"
                )

                # Prepare the message for publishing
                data = Float64MultiArray()
                data.data = [
                    # message.controls.total_force,
                    message.controls.torque_x,
                    message.controls.torque_y,
                    message.controls.torque_z,
                ]
                self.pub_control_signals.publish(data)
                self.get_logger().info(f"Published: {data.data}")
            else:
                self.get_logger().error("Error receiving message")
        except Exception as e:
            self.get_logger().error(f"Error during serial polling: {e}")

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
        self.roll_setpoint_updated = False
        self.pitch_setpoint_updated = False

    def apply_disturbance(self):
        """Apply a temporary disturbance to the roll setpoint."""
        self.get_logger().info("Applying disturbance...")
        self.roll_command -= 25.0  # Apply a 10-degree disturbance
        self.disturbance_applied = True

    def reset_disturbance(self):
        """Reset the roll setpoint after the disturbance."""
        self.get_logger().info("Resetting disturbance...")
        self.roll_command = 0.0  # Reset roll setpoint to 0 degrees
        self.disturbance_applied = False

    def disturbance_control(self):
        current_time = self.get_clock().now().seconds_nanoseconds()[0]

        # Apply disturbance at intervals
        if not self.disturbance_applied and current_time - self.last_disturbance_time >= self.disturbance_interval:
            self.apply_disturbance()
            self.last_disturbance_time = current_time

        # Reset disturbance after the duration
        if self.disturbance_applied and current_time - self.last_disturbance_time >= self.disturbance_duration:
            self.reset_disturbance()


def main(args=None):
    rclpy.init(args=args)
    node = HITLNode()
    # rclpy.spin(node)
    # rclpy.shutdown()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    loop_rate = node.create_rate(100)  # 100 Hz
    try:
        while rclpy.ok():
            node.send_to_micon()
            node.poll_serial()
            node.publish_hitl_status(True)
            # node.get_logger().info("main loop running...")

            executor.spin_once(timeout_sec=0.1)

            # time.sleep(0.01)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
