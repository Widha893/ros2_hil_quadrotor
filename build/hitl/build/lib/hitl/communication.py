import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, Range
from geometry_msgs.msg import Pose, Twist, Vector3, Quaternion
import numpy as np
from tf2_geometry_msgs import tf2_geometry_msgs
from tf2_ros import transform_broadcaster
from std_msgs.msg import Bool
from scipy.spatial.transform import Rotation as R
import math

class Communication(Node):
    def __init__(self):
        super().__init__('communication')
        self.get_logger().info("Communication node has started...")

        # Define topics
        self.imu_topic = '/simple_drone/imu/out'
        self.alt_topic = 'simple_drone/gt_pose'
        self.alt_rate_topic = '/simple_drone/gt_vel'

        # Define Quality of Service
        qos = 10

        self.imu_updated = False
        self.alt_updated = False
        self.alt_rate_updated = False

        # Create subscribers
        self.imu_subscriber = self.create_subscription(
            Imu,self.imu_topic,self.imu_callback,qos
        )
        self.alt_subscriber = self.create_subscription(
            Pose,self.alt_topic,self.alt_callback,qos
        )
        # self.alt_rate_subscriber = self.create_subscription(
        #     Twist,self.alt_rate_topic,self.alt_rate_callback,qos
        # )

    def imu_callback(self, msg: Imu):
        # Extract quaternion and convert to Euler angles
        quat = msg.orientation
        self.euler = np.degrees(self.quaternion_to_euler(quat.w, quat.x, quat.y, quat.z))
        self.angular_vel_x = msg.angular_velocity.x
        self.angular_vel_y = msg.angular_velocity.y
        self.angular_vel_z = msg.angular_velocity.z
        self.imu_updated = True
        


    def alt_callback(self, msg: Pose):
        # Extract quaternion and convert to Euler angles
        self.altitude = msg.position.z
        self.alt_updated = True
        if self.imu_updated:
            self.get_logger().info(f"IMU (Euler Angles): Roll={self.euler[0]:.2f}, Pitch={self.euler[1]:.2f}, Yaw={self.euler[2]:.2f}")
            self.get_logger().info(f"IMU (Euler Angles): Roll Vel={self.angular_vel_x:.2f}, Pitch Vel={self.angular_vel_y:.2f}, Yaw Vel={self.angular_vel_z:.2f}")
            self.get_logger().info(f"Pose: Altitude={self.altitude:.2f}")
            self.reset_flags()

    def alt_rate_callback(self,msg: Twist):
        self.altitude_rate = msg.linear.z
        self.alt_rate_updated = True
        if self.imu_updated and self.alt_updated:
            self.get_logger().info(f"IMU (Euler Angles): Roll={self.euler[0]:.2f}, Pitch={self.euler[1]:.2f}, Yaw={self.euler[2]:.2f}")
            self.get_logger().info(f"IMU (Euler Angles): Roll Vel={self.angular_vel_x:.2f}, Pitch Vel={self.angular_vel_y:.2f}, Yaw Vel={self.angular_vel_z:.2f}")
            self.get_logger().info(f"Pose: Altitude={self.altitude:.2f}, Altitude Rate={self.altitude_rate:.2f}")
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
        self.gt_pose_updated = False
        self.gt_vel_updated = False
        self.gt_acc_updated = False

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