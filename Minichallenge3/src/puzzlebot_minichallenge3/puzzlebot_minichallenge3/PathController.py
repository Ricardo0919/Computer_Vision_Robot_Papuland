import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from custom_interfaces.msg import PathPose
from rclpy.qos import qos_profile_sensor_data
import numpy as np

class PathController(Node):
    def __init__(self):
        super().__init__('PathController')

        self.pose_sub = self.create_subscription(PathPose, '/goals', self.path_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, qos_profile_sensor_data)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.points = []
        self.target_index = 0
        self.goal_threshold = 0.08
        self.finished = False

        self.kp_lin = 1.0
        self.kd_lin = 0.1
        self.kp_ang = 4.0
        self.kd_ang = 0.3

        self.prev_lin_error = 0.0
        self.prev_ang_error = 0.0
        self.prev_time = self.get_clock().now()

        self.get_logger().info("🧭 PathController PID activado")

    def path_callback(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        self.points.append((x, y))
        self.get_logger().info(f"➕ Añadido punto: ({x:.2f}, {y:.2f})")

    def odom_callback(self, msg):
        if self.finished or self.target_index >= len(self.points):
            return

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, theta = self.euler_from_quaternion(q.x, q.y, q.z, q.w)

        goal_x, goal_y = self.points[self.target_index]
        dx = goal_x - x
        dy = goal_y - y
        distance = np.hypot(dx, dy)
        target_theta = np.arctan2(dy, dx)
        ang_error = self.normalize_angle(target_theta - theta)

        now = self.get_clock().now()
        dt = (now - self.prev_time).nanoseconds * 1e-9
        self.prev_time = now

        lin_error = distance if abs(ang_error) < 0.3 else 0.0
        der_lin = (lin_error - self.prev_lin_error) / dt if dt > 0 else 0.0
        self.prev_lin_error = lin_error
        v = self.kp_lin * lin_error + self.kd_lin * der_lin

        der_ang = (ang_error - self.prev_ang_error) / dt if dt > 0 else 0.0
        self.prev_ang_error = ang_error
        w = self.kp_ang * ang_error + self.kd_ang * der_ang

        cmd = Twist()
        cmd.linear.x = np.clip(v, -0.3, 0.3)
        cmd.angular.z = np.clip(w, -1.5, 1.5)

        if distance < self.goal_threshold:
            self.get_logger().info(f"✅ Punto {self.target_index + 1} alcanzado")
            self.target_index += 1
            if self.target_index >= len(self.points):
                self.finished = True
                self.get_logger().info("🎯 Ruta completada")
                self.cmd_pub.publish(Twist())
                return

        self.cmd_pub.publish(cmd)

    def normalize_angle(self, angle):
        return np.arctan2(np.sin(angle), np.cos(angle))

    def euler_from_quaternion(self, x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = np.arctan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = np.clip(t2, -1.0, 1.0)
        pitch = np.arcsin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = np.arctan2(t3, t4)

        return roll, pitch, yaw


def main(args=None):
    rclpy.init(args=args)
    node = PathController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()