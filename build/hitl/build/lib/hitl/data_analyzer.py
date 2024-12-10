import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Pose
import numpy as np
import math
import matplotlib.pyplot as plt
import time

class SensorDataLogger(Node):
    def __init__(self):
        super().__init__('data_analyzer')

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

        # if self.altitude_topic:
        #     # Create the alt subscriber
        #     self.altitude_subscriber = self.create_subscription(
        #         Pose,
        #         self.altitude_topic,
        #         self.altitude_callback,
        #         qos
        #     )
        #     self.get_logger().info(f"Subscribed to altitude topic: {self.altitude_topic}")
        # else:
        #     self.get_logger().error("No Altitude topic defined!")

        # Matplotlib setup for real-time plotting
        self.time_data = []
        self.roll_data = []
        self.pitch_data = []
        self.yaw_data = []
        self.setpoint_data = {'roll': [], 'pitch': [], 'yaw': []}

        # Setpoints (roll, pitch, yaw) are all 0
        self.roll_setpoint = 0.0
        self.pitch_setpoint = 0.0
        self.yaw_setpoint = 0.0
        
        # Matplotlib interactive mode
        plt.ion()  # Enable interactive mode for real-time plotting
        self.fig, self.ax = plt.subplots(3, 1, figsize=(10, 6))

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
            self.get_logger().info(f"Euler Angles [Degrees]: Roll={self.roll:.2f}, Pitch={self.pitch:.2f}, Yaw={self.yaw:.2f}")
            self.update_plot()
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
    
    def altitude_callback(self, msg: Pose):
        self.altitude = msg.position.z
        self.alt_updated = True

    def update_plot(self):
        # Record the current time
        timestamp = time.time()

        # Append sensor data
        self.time_data.append(timestamp)
        self.roll_data.append(self.roll)
        self.pitch_data.append(self.pitch)
        self.yaw_data.append(self.yaw)
        
        # Append setpoints (all set to 0)
        self.setpoint_data['roll'].append(self.roll_setpoint)
        self.setpoint_data['pitch'].append(self.pitch_setpoint)
        self.setpoint_data['yaw'].append(self.yaw_setpoint)

        # Update the plots
        self.ax[0].cla()  # Clear the previous plot
        self.ax[1].cla()
        self.ax[2].cla()

        self.ax[0].plot(self.time_data, self.roll_data, label='Roll', color='blue')
        self.ax[0].plot(self.time_data, self.setpoint_data['roll'], label='Roll Setpoint', color='red', linestyle='--')
        self.ax[0].set_title('Roll vs Time')
        self.ax[0].set_xlabel('Time [s]')
        self.ax[0].set_ylabel('Roll [degrees]')
        self.ax[0].legend()

        self.ax[1].plot(self.time_data, self.pitch_data, label='Pitch', color='green')
        self.ax[1].plot(self.time_data, self.setpoint_data['pitch'], label='Pitch Setpoint', color='red', linestyle='--')
        self.ax[1].set_title('Pitch vs Time')
        self.ax[1].set_xlabel('Time [s]')
        self.ax[1].set_ylabel('Pitch [degrees]')
        self.ax[1].legend()

        self.ax[2].plot(self.time_data, self.yaw_data, label='Yaw', color='purple')
        self.ax[2].plot(self.time_data, self.setpoint_data['yaw'], label='Yaw Setpoint', color='red', linestyle='--')
        self.ax[2].set_title('Yaw vs Time')
        self.ax[2].set_xlabel('Time [s]')
        self.ax[2].set_ylabel('Yaw [degrees]')
        self.ax[2].legend()

        plt.draw()  # Redraw the plot
        plt.pause(0.1)  # Pause to update the plot

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
