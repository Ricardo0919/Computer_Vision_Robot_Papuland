import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from custom_interfaces.msg import PathPose
import numpy as np
import time
import math

def angle_diff(a, b):
    """Diferencia angular normalizada en [-π, π]"""
    diff = a - b
    while diff > math.pi:
        diff -= 2 * math.pi
    while diff < -math.pi:
        diff += 2 * math.pi
    return diff

class PathController(Node):
    def __init__(self):
        super().__init__('path_controller')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pose_sub = self.create_subscription(PathPose, '/pose', self.pose_callback, 10)

        self.points = []
        self.mode = None
        self.total_time = 0.0
        self.speed_param = 0.0

        self.state = 'WAITING'
        self.current_segment = 0
        self.start_time = None

        self.base_ang_speed = 0.3  # rad/s
        self.timer_period = 0.05
        self.timer = self.create_timer(self.timer_period, self.control_loop)

        self.angles_signo = []
        self.angles_magnitude = []
        self.distances = []
        self.turn_times = []
        self.forward_times = []
        self.lin_speeds = []
        self.num_segments = 0

        self.get_logger().info("🧭 PathController iniciado con control_loop cada 0.05s")

    def pose_callback(self, msg):
        if self.mode is None:
            if msg.time_to_reach > 0:
                self.mode = 'time'
                self.total_time = msg.time_to_reach
                self.get_logger().info(f"⏱ Modo TIME => {self.total_time:.2f}s total")
            else:
                self.mode = 'speed'
                self.speed_param = msg.linear_velocity
                self.get_logger().info(f"🏎 Modo SPEED => {self.speed_param:.2f} m/s")

        x = msg.pose.position.x
        y = msg.pose.position.y
        self.points.append((x, y))
        self.get_logger().info(f"➕ Recibido punto {len(self.points)}: ({x:.2f}, {y:.2f})")

    def setup_trajectory(self):
        self.num_segments = len(self.points) - 1
        angles = []
        distances = []

        for i in range(self.num_segments):
            p1 = np.array(self.points[i])
            p2 = np.array(self.points[i + 1])
            diff = p2 - p1
            distance = np.linalg.norm(diff)
            angle = math.atan2(diff[1], diff[0])
            distances.append(distance)
            angles.append(angle)

        self.distances = distances

        self.angles_magnitude = []
        self.angles_signo = []
        for i in range(self.num_segments):
            if i == 0:
                delta = angles[0]  # Desde orientación inicial (0 rad)
            else:
                delta = angle_diff(angles[i], angles[i - 1])
            self.angles_magnitude.append(abs(delta))
            self.angles_signo.append(np.sign(delta))

        total_dist = sum(distances)
        total_turn_time = sum([a / self.base_ang_speed for a in self.angles_magnitude])
        remaining_time = max(self.total_time - total_turn_time, 0.01)

        self.forward_times = [remaining_time * (d / total_dist) if total_dist > 0 else 0.0 for d in distances]
        self.turn_times = [a / self.base_ang_speed for a in self.angles_magnitude]
        self.lin_speeds = [d / t if t > 0 else 0.0 for d, t in zip(distances, self.forward_times)]

        self.get_logger().info("🛠 Trayectoria configurada")
        for i in range(self.num_segments):
            self.get_logger().info(f"Tramo {i+1}: {distances[i]:.2f} m, giro {self.angles_magnitude[i]:.2f} rad, t_giro {self.turn_times[i]:.2f}s, t_avance {self.forward_times[i]:.2f}s")

    def stop_robot(self):
        twist = Twist()
        self.cmd_pub.publish(twist)

    def control_loop(self):
        now = time.time()

        if self.state == 'WAITING':
            if len(self.points) >= 2 and self.mode == 'time':
                self.setup_trajectory()
                self.current_segment = 0
                self.state = 'TURN'
                self.start_time = now

        elif self.state == 'TURN':
            if self.current_segment >= self.num_segments:
                self.stop_robot()
                self.state = 'DONE'
                self.get_logger().info("🏁 Ruta completada")
                return

            turn_t = self.turn_times[self.current_segment]
            elapsed = now - self.start_time

            if elapsed < turn_t:
                twist = Twist()
                twist.angular.z = self.angles_signo[self.current_segment] * self.base_ang_speed
                self.cmd_pub.publish(twist)
            else:
                self.stop_robot()
                self.start_time = now
                self.state = 'FORWARD'

        elif self.state == 'FORWARD':
            forward_t = self.forward_times[self.current_segment]
            elapsed = now - self.start_time

            if elapsed < forward_t:
                twist = Twist()
                twist.linear.x = self.lin_speeds[self.current_segment]
                self.cmd_pub.publish(twist)
            else:
                self.stop_robot()
                self.current_segment += 1
                self.state = 'TURN'
                self.start_time = now

        elif self.state == 'DONE':
            self.stop_robot()

def main(args=None):
    rclpy.init(args=args)
    node = PathController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
