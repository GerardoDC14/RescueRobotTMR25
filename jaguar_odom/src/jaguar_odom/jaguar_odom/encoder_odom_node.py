#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import tf_transformations
import math
import re

# Robot physical parameters
WHEEL_DIAMETER = 0.18                   # in meters (18 cm)
WHEEL_CIRCUMFERENCE = math.pi * WHEEL_DIAMETER
TRACK_WIDTH = 0.47                      # in meters (47 cm)

# Complementary filter parameters
ALPHA_OMEGA = 0.6   
ALPHA_V = 0.8      

class OdomNode(Node):
    def __init__(self):
        super().__init__('odom_node')
        # Subscribers
        self.create_subscription(Imu, 'imu', self.imu_callback, 10)
        self.create_subscription(String, 'motors_info', self.encoder_callback, 10)
        # Publisher for odometry
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        # TF broadcaster for odometry
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Odometry state variables
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0  
        self.last_integration_time = None
        
        # Latest sensor measurements
        self.imu_angular_z = None   # Angular velocity from IMU (rad/s)
        self.imu_yaw = None         # Absolute yaw from IMU (radians)
        self.encoder_v_left = None  # Left wheel linear velocity (m/s, with sign)
        self.encoder_v_right = None # Right wheel linear velocity (m/s, with sign)
        
        # For encoder timestamp (in seconds)
        self.prev_encoder_time = None
        
        # Filtered outputs (EMA)
        self.filtered_v_robot = None
        self.filtered_omega = None

        # Timer for higher publication rate
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10Hz

    def imu_callback(self, msg: Imu):
        # Extract yaw and angular velocity (z axis) from the IMU quaternion.
        q = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        (roll, pitch, yaw) = tf_transformations.euler_from_quaternion(q)
        self.imu_yaw = yaw
        self.imu_angular_z = msg.angular_velocity.z

    def encoder_callback(self, msg: String):
        data_str = msg.data.strip()
        pattern = (r'\[(\d+)\s*ms\]\s*M1:\s*([-\d.]+)\s*m/s,\s*([-\d.]+)\s*RPM,\s*([FR])\s*\|\s*'
                   r'M2:\s*([-\d.]+)\s*m/s,\s*([-\d.]+)\s*RPM,\s*([FR])')
        match = re.match(pattern, data_str)
        if match:
            timestamp_ms = int(match.group(1))
            raw_v_left = float(match.group(2))
            rpm_left = float(match.group(3))
            dir_left = match.group(4)
            raw_v_right = float(match.group(5))
            rpm_right = float(match.group(6))
            dir_right = match.group(7)
            
            v_left = raw_v_left if dir_left == 'F' else -raw_v_left
            v_right = raw_v_right if dir_right == 'F' else -raw_v_right
            
            self.encoder_v_left = v_left
            self.encoder_v_right = v_right
            
            self.get_logger().info(
                f"Encoder Parsed: time={timestamp_ms} ms, "
                f"M1: {v_left:.2f} m/s, {rpm_left:.2f} RPM, {dir_left} | "
                f"M2: {v_right:.2f} m/s, {rpm_right:.2f} RPM, {dir_right}"
            )
            
            current_time = timestamp_ms / 1000.0
            if self.prev_encoder_time is None:
                self.prev_encoder_time = current_time
                return
            dt = current_time - self.prev_encoder_time
            self.prev_encoder_time = current_time
            if dt <= 0:
                self.get_logger().warn("Non-positive dt; skipping odometry update")
                return
            
            # Compute robot linear velocity from encoders 
            encoder_v_robot = (v_left + v_right) / 2.0
            # Compute encoder-derived angular velocity (rad/s)
            encoder_omega = (v_right - v_left) / TRACK_WIDTH
            
            # Complementary filtering of angular velocity:
            # Fuse encoder and IMU angular velocity if available.
            if self.imu_angular_z is not None:
                omega_fused = ALPHA_OMEGA * self.imu_angular_z + (1 - ALPHA_OMEGA) * encoder_omega
            else:
                omega_fused = encoder_omega
            
            # Apply EMA filtering to linear velocity:
            if self.filtered_v_robot is None:
                self.filtered_v_robot = encoder_v_robot
            else:
                self.filtered_v_robot = ALPHA_V * encoder_v_robot + (1 - ALPHA_V) * self.filtered_v_robot
            
            # Apply EMA filtering to angular velocity:
            if self.filtered_omega is None:
                self.filtered_omega = omega_fused
            else:
                self.filtered_omega = ALPHA_V * omega_fused + (1 - ALPHA_V) * self.filtered_omega
            
            # Integrate velocities to update pose
            if self.last_integration_time is None:
                self.last_integration_time = current_time
                return
            delta_t = current_time - self.last_integration_time
            self.last_integration_time = current_time
            
            # For orientation, if IMU yaw is available, use it; otherwise, integrate angular velocity.
            if self.imu_yaw is not None:
                self.yaw = self.imu_yaw
            else:
                self.yaw += self.filtered_omega * delta_t

            # Integrate linear velocity to update x and y
            delta_x = self.filtered_v_robot * delta_t * math.cos(self.yaw)
            delta_y = self.filtered_v_robot * delta_t * math.sin(self.yaw)
            self.x += delta_x
            self.y += delta_y

    def timer_callback(self):
        if self.filtered_v_robot is not None and self.filtered_omega is not None:
            current_ros_time = self.get_clock().now()
            self.publish_odometry(current_ros_time, self.filtered_v_robot, self.filtered_omega)

    def publish_odometry(self, current_ros_time, v_robot, omega):
        odom_msg = Odometry()
        odom_msg.header.stamp = current_ros_time.to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_footprint"

        # Set the pose
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        q = tf_transformations.quaternion_from_euler(0.0, 0.0, self.yaw)
        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]

        # Set the twist (velocities)
        odom_msg.twist.twist.linear.x = v_robot
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.angular.z = omega

        # Publish odometry
        self.odom_pub.publish(odom_msg)
        self.get_logger().info(
            f"Odometry: x={self.x:.2f}, y={self.y:.2f}, yaw={self.yaw:.2f}, "
            f"v={v_robot:.2f} m/s, ω={omega:.2f} rad/s"
        )

        # Broadcast the TF transform from odom → base_footprint
        transform = TransformStamped()
        transform.header.stamp = current_ros_time.to_msg()
        transform.header.frame_id = "odom"
        transform.child_frame_id = "base_footprint"
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = 0.0
        transform.transform.rotation = odom_msg.pose.pose.orientation

        self.tf_broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    node = OdomNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()