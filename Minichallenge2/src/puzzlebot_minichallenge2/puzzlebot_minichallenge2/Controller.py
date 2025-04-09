#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time
import math

class Controller(Node):
    def __init__(self):
        super().__init__('square_controller')

        self.declare_parameter('side_length', 2.0)
        self.declare_parameter('mode', 'time')
        self.declare_parameter('linear_speed', 0.2)
        self.declare_parameter('total_time', 30.0)

        self.get_params()

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pose_sub = self.create_subscription(Odometry, '/ground_truth', self.pose_callback, 10)

        self.timer = self.create_timer(0.1, self.loop)
        self.state = 'START'
        self.count = 0
        self.t_start = time.time()

        self.current_pose = None
        s = self.side_length
        self.target_points = [(0, 0), (s, 0), (s, s), (0, s)]
        self.real_positions = []

        self.get_logger().info('SquareController iniciado.')

    def pose_callback(self, msg):
        self.current_pose = msg.pose.pose

    def get_params(self):
        self.side_length = self.get_parameter('side_length').value
        self.mode = self.get_parameter('mode').value.lower().strip()
        self.linear_speed = self.get_parameter('linear_speed').value
        self.total_time = self.get_parameter('total_time').value

        if self.mode not in ['speed', 'time']:
            self.get_logger().error(f"Modo inválido: '{self.mode}'. Debe ser 'speed' o 'time'.")
            rclpy.shutdown()
            return

        if self.mode == 'time':
            straight_time = self.total_time / 8
            turn_time = self.total_time / 8
            self.linear_speed = self.side_length / straight_time
            self.angular_speed = (math.pi / 2) / turn_time
        else:
            self.t_straight = self.side_length / self.linear_speed
            self.angular_speed = math.pi / 4
            self.t_turn = (math.pi / 2) / self.angular_speed
            self.total_time = 4 * (self.t_straight + self.t_turn)

        self.t_straight = self.side_length / self.linear_speed
        self.t_turn = (math.pi / 2) / self.angular_speed

        self.get_logger().info(
            f"[PARAMS] mode={self.mode} | total_time={self.total_time:.2f} s | "
            f"velocidad lineal={self.linear_speed:.2f} m/s | velocidad angular={self.angular_speed:.2f} rad/s"
        )

    def loop(self):
        now = time.time()
        elapsed = now - self.t_start
        self.get_params()
        twist = Twist()

        if self.state == 'START':
            if self.current_pose is not None:
                x0 = self.current_pose.position.x
                y0 = self.current_pose.position.y
            else:
                x0, y0 = (0.0, 0.0)
                self.get_logger().warn("No se recibió pose; usando (0,0) para p1")

            self.real_positions.append((x0, y0))
            self.get_logger().info(f"Posición inicial (p1): ({x0:.2f}, {y0:.2f})")

            self.state = 'STRAIGHT'
            self.t_start = now

        elif self.state == 'STRAIGHT':
            if elapsed < self.t_straight:
                twist.linear.x = self.linear_speed
            else:
                self.state = 'TURN'
                self.t_start = now

                if self.current_pose:
                    x = self.current_pose.position.x
                    y = self.current_pose.position.y
                else:
                    x, y = (0.0, 0.0)
                    self.get_logger().warn("No se recibió pose al terminar lado; usando (0,0)")

                self.real_positions.append((x, y))
                self.count += 1
                self.get_logger().info(f"Lado {self.count} completado en ({x:.2f}, {y:.2f})")

        elif self.state == 'TURN':
            if elapsed < self.t_turn:
                twist.angular.z = self.angular_speed
            else:
                if self.count >= 4:
                    self.state = 'STOP'
                    self.get_logger().info("Trayectoria cuadrada completada.")
                else:
                    self.state = 'STRAIGHT'
                self.t_start = now

        elif self.state == 'STOP':
            self.publisher.publish(Twist())

            # Agregar punto final
            if self.current_pose:
                x = self.current_pose.position.x
                y = self.current_pose.position.y
            else:
                x, y = (0.0, 0.0)
                self.get_logger().warn("No se recibió pose final; usando (0,0) para pf")

            self.real_positions.append((x, y))  # Este es pf
            self.print_and_save_errors()
            rclpy.shutdown()
            return

        self.publisher.publish(twist)

    def print_and_save_errors(self):
        def fmt(val):
            return 0.0 if abs(val) < 0.005 else round(val, 2)

        lines = ["\nTabla de Errores (d_error en metros):\n"]
        lines.append("{:<4} {:>12} {:>18} {:>10}".format("Punto", "Objetivo", "Posición real", "Error"))

        for i in range(4):
            px, py = self.target_points[i]
            rx, ry = self.real_positions[i]
            err = math.sqrt((rx - px)**2 + (ry - py)**2)
            lines.append(f"p{i+1:<3} ({px:5.1f},{py:5.1f})  ({fmt(rx):8.2f},{fmt(ry):8.2f})   {err:8.3f}")

        rx_last, ry_last = self.real_positions[4]
        err_pf = math.sqrt((rx_last - 0.0)**2 + (ry_last - 0.0)**2)
        lines.append(f"pf   (  0.0,  0.0)  ({fmt(rx_last):8.2f},{fmt(ry_last):8.2f})   {err_pf:8.3f}")

        lines.append("")
        result = "\n".join(lines)
        with open("errors.txt", "w") as f:
            f.write(result)

        self.get_logger().info(result)


def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
