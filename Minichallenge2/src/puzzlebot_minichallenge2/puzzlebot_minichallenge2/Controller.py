#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time
import math

class SquareController(Node):
    def __init__(self):
        super().__init__('square_controller')

        # Parámetros configurables por el usuario
        self.declare_parameter('side_length', 2.0)
        self.declare_parameter('mode', 'time')  # 'speed' o 'time'
        self.declare_parameter('linear_speed', 0.2)
        self.declare_parameter('total_time', 30.0)
        self.declare_parameter('turn_direction', 'right')  # Nuevo parámetro

        self.get_params()

        # Comunicación
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pose_sub = self.create_subscription(Odometry, '/ground_truth', self.pose_callback, 10)

        # Estados
        self.timer = self.create_timer(0.1, self.loop)  # 10 Hz
        self.state = 'START'
        self.count = 0
        self.t_start = time.time()

        # Datos de posición
        self.current_pose = None
        s = self.side_length
        self.target_points = [(0, 0), (s, 0), (s, s), (0, s)]
        self.errors = []

        self.get_logger().info('SquareController iniciado.')

    def pose_callback(self, msg):
        self.current_pose = msg.pose.pose

    def get_params(self):
        self.side_length = self.get_parameter('side_length').value
        self.mode = self.get_parameter('mode').value.lower().strip()
        self.linear_speed = self.get_parameter('linear_speed').value
        self.total_time = self.get_parameter('total_time').value
        self.turn_direction = self.get_parameter('turn_direction').value.lower().strip()

        if self.mode not in ['speed', 'time']:
            self.get_logger().error(f"Modo inválido: '{self.mode}'. Debe ser 'speed' o 'time'.")
            rclpy.shutdown()
            return

        if self.turn_direction not in ['left', 'right']:
            self.get_logger().error(f"Dirección inválida: '{self.turn_direction}'. Usa 'left' o 'right'.")
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
            f"velocidad lineal={self.linear_speed:.2f} m/s | velocidad angular={self.angular_speed:.2f} rad/s | "
            f"dirección de giro: {self.turn_direction}"
        )

    def calculate_error(self, target):
        if self.current_pose is None:
            self.get_logger().warn('No se recibió pose aún, error no calculado.')
            return None

        x_real = self.current_pose.position.x
        y_real = self.current_pose.position.y
        x_target, y_target = target
        error = math.sqrt((x_real - x_target) ** 2 + (y_real - y_target) ** 2)
        return error

    def save_errors_to_txt(self):
        lines = ["\nTabla de Errores (d_error en metros):\n"]
        lines.append("{:<5} {:>10} {:>10}".format("Punto", "Objetivo", "Error"))
        for i, error in enumerate(self.errors):
            pt = f"p{i+1}"
            target = self.target_points[i]
            lines.append("{:<5} ({:>4.1f}, {:>4.1f}) {:>10.3f}".format(pt, target[0], target[1], error))
        lines.append("")
        result = "\n".join(lines)

        with open("errors.txt", "w") as f:
            f.write(result)

        self.get_logger().info("Tabla de errores guardada en 'errors.txt'.")
        self.get_logger().info("\n" + result)

    def loop(self):
        now = time.time()
        elapsed = now - self.t_start
        self.get_params()
        twist = Twist()

        if self.state == 'START':
            self.get_logger().info('Empezando recorrido en cuadrado...')
            self.state = 'STRAIGHT'
            self.t_start = now

        elif self.state == 'STRAIGHT':
            if elapsed < self.t_straight:
                twist.linear.x = self.linear_speed
            else:
                self.state = 'TURN'
                self.t_start = now
                error = self.calculate_error(self.target_points[self.count])
                if error is not None:
                    self.errors.append(error)
                    self.get_logger().info(f"Lado {self.count+1} completado. Error = {error:.3f} m")
                else:
                    self.errors.append(-1)
                self.get_logger().info(f"Iniciando giro {self.count+1}...")

        elif self.state == 'TURN':
            if elapsed < self.t_turn:
                direction = 1.0 if self.turn_direction == 'left' else -1.0
                twist.angular.z = direction * self.angular_speed
            else:
                self.count += 1
                if self.count >= 4:
                    self.state = 'STOP'
                    self.get_logger().info('Trayectoria cuadrada completada.')
                else:
                    self.state = 'STRAIGHT'
                    self.t_start = now

        elif self.state == 'STOP':
            self.publisher.publish(Twist())
            self.save_errors_to_txt()
            rclpy.shutdown()
            return

        self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = SquareController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
