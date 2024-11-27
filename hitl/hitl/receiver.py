import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import serial
import struct
from std_msgs.msg import Bool
from hitl.messages_pb2 import msg as HWIL_msg  # Import the HWIL_msg class from your Protobuf file

# Constants
STX = 0xFE
MAX_MESSAGE_SIZE = 1024  # Maximum buffer size, adjust if needed

class SerialReceiverNode(Node):
    def __init__(self):
        super().__init__('receiver')

        # ROS 2 publisher
        self.publisher_ = self.create_publisher(Float64MultiArray, '~/control_data', 10)
        self.pubHITL = self.create_publisher(Bool, '~/hitl', 1024)

        # Serial port configuration
        self.port = '/dev/ttyACM0'
        self.baud_rate = 9600
        self.serial_port = None

        # HITL mode
        self.mHITL(True)

        # Timer for polling serial data
        self.timer_period = 0.1  # 10 Hz
        self.timer = self.create_timer(self.timer_period, self.poll_serial)

        # Initialize serial port
        try:
            self.serial_port = serial.Serial(self.port, self.baud_rate, timeout=1)
            self.get_logger().info("Serial port initialized successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            raise

    def poll_serial(self):
        """
        Poll the serial port for data and publish it to the ROS 2 topic
        """
        try:
            message = self.receive_message()
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
                    message.controls.total_force,
                    message.controls.torque_x,
                    message.controls.torque_y,
                    message.controls.torque_z,
                ]
                self.publisher_.publish(data)
                self.get_logger().info(f"Published: {data.data}")
        except Exception as e:
            self.get_logger().error(f"Error during serial polling: {e}")

    def receive_message(self):
        """
        Receive and decode a Protobuf message from the serial port
        """
        try:
            # Wait for the start byte (STX)
            while True:
                byte = self.serial_port.read(1)
                if not byte:
                    raise TimeoutError("Timeout waiting for STX")
                if byte[0] == STX:
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
            hwil_message = HWIL_msg()
            hwil_message.ParseFromString(message_data)
            return hwil_message

        except Exception as e:
            self.get_logger().error(f"Error receiving message: {e}")
            return None
        
    def mHITL(self, on: bool):
        """
        Turn on/off position control
        :param on: True to turn on position control, False to turn off
        """
        self.hitl_mode = Bool()
        self.hitl_mode.data = on
        self.pubHITL.publish(self.hitl_mode)
        return True


def main(args=None):
    rclpy.init(args=args)

    # Create the node
    receiver_node = SerialReceiverNode()

    try:
        # Spin the node to process callbacks
        rclpy.spin(receiver_node)
    except KeyboardInterrupt:
        receiver_node.get_logger().info("Shutting down receiver node.")
    finally:
        receiver_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
