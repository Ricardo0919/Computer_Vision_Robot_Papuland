#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import numpy as np
from rclpy.qos import qos_profile_sensor_data

class OrientationComparator(Node):
    def __init__(self):
        super().__init__('OrientationComparator')
        self.theta_odom = None
        self.theta_gt = None

        self.create_subscription(Odometry, '/odom', self.odom_callback, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/ground_truth', self.gt_callback, 10)

        self.timer = self.create_timer(0.2, self.compare_orientations)

    def odom_callback(self, msg):
        q = msg.pose.pose.orientation
        self.theta_odom = self.quaternion_to_yaw(q)

    def gt_callback(self, msg):
        q = msg.pose.pose.orientation
        self.theta_gt = self.quaternion_to_yaw(q)

    def quaternion_to_yaw(self, q):
        # Extrae yaw del quaternion
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return np.arctan2(siny_cosp, cosy_cosp)

    def compare_orientations(self):
        if self.theta_odom is not None and self.theta_gt is not None:
            diff = self.normalize_angle(self.theta_odom - self.theta_gt)
            self.get_logger().info(f"🧭 θ_odom = {np.degrees(self.theta_odom):.1f}° | θ_GT = {np.degrees(self.theta_gt):.1f}° | Δθ = {np.degrees(diff):+.1f}°")

    def normalize_angle(self, angle):
        return np.arctan2(np.sin(angle), np.cos(angle))

def main(args=None):
    rclpy.init(args=args)
    node = OrientationComparator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
