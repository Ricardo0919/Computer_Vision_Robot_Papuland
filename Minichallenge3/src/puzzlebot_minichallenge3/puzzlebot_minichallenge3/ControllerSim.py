#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import numpy as np

class Controller(Node):
    def __init__(self):
        super().__init__('Controller')

        self.declare_parameter('side_length', 2.0)
        self.side_length = self.get_parameter('side_length').value

        # PID gains
        self.kp_lin = 1.0
        self.ki_lin = 0.0
        self.kd_lin = 0.1

        self.kp_ang = 4.0
        self.ki_ang = 0.0
        self.kd_ang = 0.3

        self.prev_lin_error = 0.0
        self.int_lin_error = 0.0
        self.prev_ang_error = 0.0
        self.int_ang_error = 0.0
        self.prev_time = self.get_clock().now()

        # Ruta cuadrada
        L = self.side_length
        self.goals = [
            (L, 0.0),
            (L, L),
            (0.0, L),
            (0.0, 0.0)
        ]
        self.goal_index = 0
        self.goal_threshold = 0.1
        self.finished = False
        self.aligning_final = False
        self.results = []

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/ground_truth', self.odom_callback, 10)

        self.get_logger().info(f"PID controller iniciado con side_length = {L} m (usando /ground_truth)")

    def odom_callback(self, msg):
        if self.finished:
            return

        # Posición actual
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, theta = self.euler_from_quaternion(q.x, q.y, q.z, q.w)

        now = self.get_clock().now()
        dt = (now - self.prev_time).nanoseconds * 1e-9
        self.prev_time = now

        cmd = Twist()

        if self.aligning_final:
            # Alinear al final hacia el punto (1, 0)
            target_theta = 0.0
            ang_error = self.normalize_angle(target_theta - theta)

            self.int_ang_error += ang_error * dt
            der_ang = (ang_error - self.prev_ang_error) / dt if dt > 0 else 0.0
            self.prev_ang_error = ang_error

            w = self.kp_ang * ang_error + self.ki_ang * self.int_ang_error + self.kd_ang * der_ang
            cmd.angular.z = np.clip(w, -1.5, 1.5)

            self.publisher.publish(cmd)

            if abs(ang_error) < 0.03:
                self.get_logger().info("🎯 Alineación final completada. ✅ Ruta completamente finalizada.")
                self.publisher.publish(Twist())
                self.finished = True
                return

            return  # Esperar a terminar alineación

        # Posición objetivo
        goal_x, goal_y = self.goals[self.goal_index]
        dx = goal_x - x
        dy = goal_y - y
        distance = np.hypot(dx, dy)
        target_theta = np.arctan2(dy, dx)
        ang_error = self.normalize_angle(target_theta - theta)

        # PID lineal
        lin_error = distance if abs(ang_error) < 0.2 else 0.0
        self.int_lin_error += lin_error * dt
        der_lin = (lin_error - self.prev_lin_error) / dt if dt > 0 else 0.0
        self.prev_lin_error = lin_error
        v = self.kp_lin * lin_error + self.ki_lin * self.int_lin_error + self.kd_lin * der_lin

        # PID angular
        self.int_ang_error += ang_error * dt
        der_ang = (ang_error - self.prev_ang_error) / dt if dt > 0 else 0.0
        self.prev_ang_error = ang_error
        w = self.kp_ang * ang_error + self.ki_ang * self.int_ang_error + self.kd_ang * der_ang

        cmd.linear.x = np.clip(v, -0.3, 0.3)
        cmd.angular.z = np.clip(w, -1.5, 1.5)

        # Verificar llegada
        if distance < self.goal_threshold:
            err_x = goal_x - x
            err_y = goal_y - y
            self.results.append({
                "objetivo": (goal_x, goal_y),
                "pos": (x, y),
                "err_x": err_x,
                "err_y": err_y,
                "dist": distance
            })
            self.get_logger().info(
                f"[Resultado] Objetivo {self.goal_index + 1}: Objetivo=({goal_x:.2f}, {goal_y:.2f}), "
                f"Llegó=({x:.2f}, {y:.2f}), Error=({err_x:.2f}, {err_y:.2f}), Dist={distance:.2f}"
            )

            self.goal_index += 1
            if self.goal_index >= len(self.goals):
                self.get_logger().info("🧭 Última esquina alcanzada, iniciando alineación final...")
                self.aligning_final = True
                self.int_ang_error = 0.0
                self.prev_ang_error = 0.0
                self.int_lin_error = 0.0
                self.prev_lin_error = 0.0
                self.print_summary()
                return

            self.get_logger().info(f"Llegó a esquina {self.goal_index}, continuando...")
            self.int_lin_error = 0.0
            self.prev_lin_error = 0.0
            self.int_ang_error = 0.0
            self.prev_ang_error = 0.0

        self.publisher.publish(cmd)

    def print_summary(self):
        self.get_logger().info("\n📊 Resumen de posiciones alcanzadas:\n")
        self.get_logger().info(f"{'Esquina':<8} | {'Objetivo':<18} | {'Llegó a':<18} | {'Error X':<8} | {'Error Y':<8} | {'Dist':<6}")
        self.get_logger().info("-" * 80)
        for i, r in enumerate(self.results):
            self.get_logger().info(
                f"{i+1:<8} | ({r['objetivo'][0]:.2f}, {r['objetivo'][1]:.2f})     | "
                f"({r['pos'][0]:.2f}, {r['pos'][1]:.2f})     | {r['err_x']:+.2f}     | "
                f"{r['err_y']:+.2f}     | {r['dist']:.2f}"
            )

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
    node = Controller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
