import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Pose, Twist, Vector3, Quaternion
import numpy as np
from tf2_geometry_msgs import tf2_geometry_msgs
from tf2_ros import transform_broadcaster
from std_msgs.msg import Bool
from scipy.spatial.transform import Rotation as R

class Communication(Node):
    def __init__(self):
        super().__init__('communication')
        self.get_logger().info("Communication node has started...")
        
        self.velocity_xy = []
        self.acceleration_xy = []

        # Define topics
        self.imu_topic = '/simple_drone/imu/out'
        self.gt_pose_topic = '/simple_drone/gt_pose'
        self.gt_vel_topic = '/simple_drone/gt_vel'
        self.gt_acc_topic = '/simple_drone/gt_acc'

        # Define Quality of Service
        qos = 10

        self.imu_updated = False
        self.gt_pose_updated = False
        self.gt_vel_updated = False
        self.gt_acc_updated = False

        # Create subscribers
        self.imu_subscriber = self.create_subscription(
            Imu,self.imu_topic,self.imu_callback,qos
        )
        self. gt_pose_subscriber = self.create_subscription(
            Pose,self.gt_pose_topic,self.gt_pose_callback,qos
        )
        self.gt_vel_subscriber = self.create_subscription(
            Twist,self.gt_vel_topic,self.gt_vel_callback,qos
        )
        self.gt_acc_subscriber = self.create_subscription(
            Twist,self.gt_acc_topic,self.gt_acc_callback,qos
        )

    def imu_callback(self, msg: Imu):
        # Extract quaternion and convert to Euler angles
        quat = msg.orientation
        self.euler = self.quaternion_to_euler(quat)
        self.imu_updated = True

    def gt_pose_callback(self, msg: Pose):
        self.pos_x = msg.position.x
        self.pos_y = msg.position.y
        self.pos_z = msg.position.z
        self.gt_pose_updated = True

    def gt_vel_callback(self, msg: Twist):
        self.velocity_xy = self.rotate_to_body_frame(msg.linear)
        self.velicity_z = msg.linear.z
        self.gt_vel_updated = True

    def gt_acc_callback(self, msg: Twist):
        self.acceleration_xy = msg.linear.x, msg.linear.y
        self.gt_acc_updated = True
        if self.imu_updated and self.gt_pose_updated and self.gt_vel_updated and self.gt_acc_updated:
            self.get_logger().info(f"IMU (Euler Angles): Roll={self.euler[0]:.2f}, Pitch={self.euler[1]:.2f}, Yaw={self.euler[2]:.2f}")
            self.get_logger().info(f"Position: x={self.pos_x:.2f}, y={self.pos_y:.2f}, z={self.pos_z:.2f}")
            self.get_logger().info(f"Velocity x (Body Frame): {self.velocity_xy[0]}")
            self.get_logger().info(f"Velocity y (Body Frame): {self.velocity_xy[1]}")
            self.get_logger().info(f"Acceleration x (Body Frame): {self.acceleration_xy[0]}")
            self.get_logger().info(f"Acceleration y (Body Frame): {self.acceleration_xy[1]}")
            self.reset_flags()
        

    def rotate_to_body_frame(self, msg):
        vector = [msg.x, msg.y, msg.z]
        heading_quaternion = R.from_euler('z', self.euler[2], degrees=False).as_quat()
        heading_rot = R.from_quat(heading_quaternion)
        return heading_rot.inv().apply(vector)

    def quaternion_to_euler(self, quat):
        """Convert a quaternion to Euler angles (roll, pitch, yaw)"""
        q = [quat.w, quat.x, quat.y, quat.z]
        sinr_cosp = 2 * (q[0] * q[1] + q[2] * q[3])
        cosr_cosp = 1 - 2 * (q[1]**2 + q[2]**2)
        roll = np.arctan2(sinr_cosp, cosr_cosp)
        sinp = 2 * (q[0] * q[2] - q[3] * q[1])
        pitch = np.arcsin(np.clip(sinp, -1.0, 1.0))
        siny_cosp = 2 * (q[0] * q[3] + q[1] * q[2])
        cosy_cosp = 1 - 2 * (q[2]**2 + q[3]**2)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
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