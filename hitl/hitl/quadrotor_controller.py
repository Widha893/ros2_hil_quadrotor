import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Imu, Range
from std_msgs.msg import Float64MultiArray, Bool
import numpy as np
import math
import matplotlib.pyplot as plt
import csv
import os
import time

DT = 0.1
MAX_VALUE = 25.0
MAX_VALUE_YAW = 10.0

class QuadrotorController(Node):
    def __init__(self):
        super().__init__('quadrotor_controller')
        self.imu_topic = '/simple_drone/imu/out'
        self.alt_topic = '/simple_drone/sonar/out'
        self.roll_command_topic = '/simple_drone/roll_command'
        self.pitch_command_topic = '/simple_drone/pitch_command'

        self.gain_roll = 5.477 # 7.071
        self.gain_p = 3.027 # 1.740
        self.gain_pitch = 5.196 # 6.325
        self.gain_q = 3.341 # 1.469
        self.gain_yaw = 3.000 # 6.856
        self.gain_r = 1.079 # 1.729

        self.prev_quat = None
        self.prev_time = None
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.setpoint_roll = 0.0
        self.current_setpoint_roll = 0.0
        self.last_setpoint_roll = 0.0
        self.setpoint_pitch = 0.0
        self.current_setpoint_pitch = 0.0
        self.last_setpoint_pitch = 0.0
        self.setpoint_yaw = 0.0
        self.current_setpoint_yaw = 0.0
        self.last_setpoint_yaw = 0.0
        self.control_roll = 0.0
        self.control_pitch = 0.0
        self.control_yaw = 0.0
        self.angular_velocity = [0.0, 0.0, 0.0]

        self.disturbance_interval = 18.0  # seconds
        self.disturbance_duration = 2.0  # seconds
        self.disturbance_applied = False
        self.last_disturbance_time = self.get_clock().now().seconds_nanoseconds()[0]

        # Data logging for plotting
        self.time_data = []
        self.roll_data = []
        self.pitch_data = []
        self.yaw_data = []
        self.setpoint_data = []  # Always log 0 for the setpoint
        self.start_time = None

        self.csv_filename = '/home/widha893/data_log_roll.csv'
        self.fieldnames = ['timestamp', 'roll']
        
        if not os.path.exists(self.csv_filename):
            with open(self.csv_filename, mode='w', newline='') as file:
                writer = csv.DictWriter(file, fieldnames=self.fieldnames)
                writer.writeheader()
        # Create publishers
        self.pub_control_signals = self.create_publisher(Float64MultiArray, '~/drone_control_signals', 10)
        self.pub_status = self.create_publisher(Bool, '~/controller_status', 10)

        self.imu_subscriber = self.create_subscription(Imu, self.imu_topic, self.imu_callback, 10)
        # self.alt_subscriber = self.create_subscription(Range, self.alt_topic, self.alt_callback, 10)

    def quaternion_to_roll(self, q_w, q_x, q_y, q_z):
        return math.atan2(2 * (q_w * q_x + q_y * q_z), 1 - 2 * (q_x**2 + q_y**2))
    
    def quaternion_to_pitch(self, q_w, q_x, q_y, q_z):
        return math.asin(2 * (q_w * q_y - q_z * q_x))
    
    def quaternion_to_yaw(self, q_w, q_x, q_y, q_z):
        return math.atan2(2 * (q_w * q_z + q_x * q_y), 1 - 2 * (q_y**2 + q_z**2))
    
    def quaternion_to_angular_velocity(self, q1, q0, dt):
        """
        Calculate angular velocity (deg/s) from two quaternions at different times.

        Parameters:
        - q1: List or array [w1, x1, y1, z1] at time t1
        - q0: List or array [w0, x0, y0, z0] at time t0
        - dt: Time step (t1 - t0) in seconds

        Returns:
        - Angular velocity [wx, wy, wz] in degrees/second
        """
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
    
    def imu_callback(self, msg: Imu):
        # Extract quaternion and convert to Euler angles
        quat = msg.orientation

        current_quat = [quat.w, quat.x, quat.y, quat.z]
        current_time = self.get_clock().now().to_msg().sec + self.get_clock().now().to_msg().nanosec * 1e-9

        self.roll = np.degrees(self.quaternion_to_roll(quat.w, quat.x, quat.y, quat.z))
        self.pitch = np.degrees(self.quaternion_to_pitch(quat.w, quat.x, quat.y, quat.z))
        self.yaw = np.degrees(self.quaternion_to_yaw(quat.w, quat.x, quat.y, quat.z))

        if self.prev_quat is not None and self.prev_time is not None:
            dt = current_time - self.prev_time
            self.angular_velocity = self.quaternion_to_angular_velocity(current_quat, self.prev_quat, dt)

        self.time_data.append(current_time)
        self.roll_data.append(self.roll)
        self.pitch_data.append(self.pitch)
        self.yaw_data.append(self.angular_velocity[2])
        self.setpoint_data.append(0.0)

        self.save_to_csv()
        
        self.prev_quat = current_quat
        self.prev_time = current_time

    def alt_callback(self, msg: Range):
        self.altitude = msg.range
        self.control_system()
        self.publish_controller_status(True)
        
    def control_system(self):
        current_time = self.get_clock().now().seconds_nanoseconds()[0] + \
                       self.get_clock().now().seconds_nanoseconds()[1] * 1e-9
        
        # Hitung dt (delta time) dalam detik
        if not hasattr(self, 'previous_time'):
            self.previous_time = current_time
        dt = current_time - self.previous_time
        self.previous_time = current_time

        self.last_setpoint_roll = self.current_setpoint_roll
        self.current_setpoint_roll = self.setpoint_roll

        self.last_setpoint_pitch = self.current_setpoint_pitch
        self.current_setpoint_pitch = self.setpoint_pitch

        self.last_setpoint_yaw = self.current_setpoint_yaw
        self.current_setpoint_yaw = self.setpoint_yaw

        # Hitung perubahan setpoint per detik
        if dt > 0:
            setpoint_roll_rate = (self.current_setpoint_roll - self.last_setpoint_roll) / dt
            setpoint_pitch_rate = (self.current_setpoint_pitch - self.last_setpoint_pitch) / dt
            setpoint_yaw_rate = (self.current_setpoint_yaw - self.last_setpoint_yaw) / dt
        else:
            setpoint_roll_rate = 0.0
            setpoint_pitch_rate = 0.0
            setpoint_yaw_rate = 0.0

        # Apply disturbance at intervals
        if not self.disturbance_applied and current_time - self.last_disturbance_time >= self.disturbance_interval:
            self.apply_disturbance()
            self.last_disturbance_time = current_time

        # Reset disturbance after the duration
        if self.disturbance_applied and current_time - self.last_disturbance_time >= self.disturbance_duration:
            self.reset_disturbance()

        error_roll = self.setpoint_roll - self.roll
        error_pitch = self.setpoint_pitch - self.pitch
        error_yaw = self.setpoint_yaw - self.yaw

        p_roll = self.gain_roll * error_roll
        d_roll = self.gain_p * (setpoint_roll_rate-self.angular_velocity[0])

        p_pitch = self.gain_pitch * error_pitch
        d_pitch = self.gain_q * (setpoint_pitch_rate-self.angular_velocity[1])

        p_yaw = self.gain_yaw * error_yaw
        d_yaw = self.gain_r * (setpoint_yaw_rate-self.angular_velocity[2])

        self.control_roll = p_roll + d_roll
        self.control_pitch = p_pitch + d_pitch
        self.control_yaw = p_yaw + d_yaw

        self.control_roll = np.clip(self.control_roll, -MAX_VALUE, MAX_VALUE)
        self.control_pitch = np.clip(self.control_pitch, -MAX_VALUE, MAX_VALUE)
        self.control_yaw = np.clip(self.control_yaw, -MAX_VALUE_YAW, MAX_VALUE_YAW)

        control = Float64MultiArray()
        control.data = [self.control_roll, self.control_pitch, self.control_yaw]
        self.pub_control_signals.publish(control)
        self.get_logger().info(
            f"Control signals - roll: {self.control_roll:.2f}, "
            f"pitch: {self.control_pitch:.2f}, "
            f"yaw: {self.control_yaw:.2f}"
        )
        self.get_logger().info(
            f"Sensor data - roll: {self.roll:.2f}, "
            f"pitch: {self.pitch:.2f}, "
            f"yaw: {self.yaw:.2f}"
        )

    def publish_controller_status(self, status):
        # Publish the HITL status as a Boolean value
        controller_status_msg = Bool()
        controller_status_msg.data = status
        self.pub_status.publish(controller_status_msg)

    def apply_disturbance(self):
        """Apply a temporary disturbance to the roll setpoint."""
        self.get_logger().info("Applying disturbance...")
        self.setpoint_roll -= 25.0  # Apply a 10-degree disturbance
        self.disturbance_applied = True

    def reset_disturbance(self):
        """Reset the roll setpoint after the disturbance."""
        self.get_logger().info("Resetting disturbance...")
        self.setpoint_roll = 0.0  # Reset roll setpoint to 0 degrees
        self.disturbance_applied = False

    def plot_response(self):
        """Plot the roll response with a 5% tolerance band."""
        
        plt.figure()
        plt.plot(self.time_data, self.roll_data, label='Roll Angle (°)')
        plt.axhline(0.0, color='r', linestyle='--', label='Setpoint (0°)')
        
        # Add tolerance band
        plt.axhline(1.5, color='g', linestyle='--', label='+5% Tolerance')
        plt.axhline(-1.5, color='g', linestyle='--', label='-5% Tolerance')
        
        plt.xlabel('Time (s)')
        plt.ylabel('Roll Angle (°)')
        plt.title('Roll Response with 5% Tolerance')
        plt.legend()
        plt.grid()
        plt.show()

    def save_to_csv(self):
        timestamp = int(time.time() * 1e3)  # timestamp in microseconds
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
            })

def main(args=None):    
    rclpy.init(args=args)
    node = QuadrotorController()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.1)
            node.control_system()
            node.publish_controller_status(True)
    except KeyboardInterrupt:
        node.get_logger().info("Plotting reponse...")
        node.plot_response()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()