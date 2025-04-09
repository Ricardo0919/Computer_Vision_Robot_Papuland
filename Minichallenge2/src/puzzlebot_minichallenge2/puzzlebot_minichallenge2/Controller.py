#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import numpy as np

class Controller(Node):
    def __init__(self):
        super().__init__('square_controller')

        self.declare_parameter('side_length', 2.0)
        self.declare_parameter('mode', 'time')
        self.declare_parameter('linear_speed', 0.2)
        self.declare_parameter('total_time', 90.0)

        self.get_params()

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.loop)

        self.state = 'START'
        self.count = 0
        self.t_start = time.time()

        self.get_logger().info('SquareController iniciado.')

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
            self.angular_speed = (np.pi / 2) / turn_time
        else:
            self.t_straight = self.side_length / self.linear_speed
            self.angular_speed = np.pi / 4
            self.t_turn = (np.pi / 2) / self.angular_speed
            self.total_time = 4 * (self.t_straight + self.t_turn)

        self.t_straight = self.side_length / self.linear_speed
        self.t_turn = (np.pi / 2) / self.angular_speed

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
            self.get_logger().info("Inicio del recorrido.")
            self.state = 'STRAIGHT'
            self.t_start = now

        elif self.state == 'STRAIGHT':
            if elapsed < self.t_straight:
                twist.linear.x = self.linear_speed
            else:
                self.state = 'TURN'
                self.t_start = now
                self.count += 1
                self.get_logger().info(f"Lado {self.count} completado.")

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
            rclpy.shutdown()
            return

        self.publisher.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
