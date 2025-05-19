#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import numpy as np

class LineController(Node):
    def __init__(self):
        super().__init__('line_controller')
        self.error = 0.0
        self.kp_base = 0.004  # Ganancia proporcional base
        self.kd = 0.0015      # Ganancia derivativa constante
        self.prev_error = 0.0
        self.prev_time = self.get_clock().now()
        self.valid_error = False

        # Suscripción al error de la línea
        self.sub = self.create_subscription(Float32, '/line_follower_data', self.error_callback, 10)
        
        # Publicador de comandos de velocidad
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        dt = 0.05  # Frecuencia de control (20 Hz)
        self.timer = self.create_timer(dt, self.timer_callback)

        self.get_logger().info('🚗 LineController (PD Adaptativo) Node iniciado sin odometría')

    def error_callback(self, msg):
        # ←--- usar np.isnan para saber si llegó un NaN
        if np.isnan(msg.data):
            self.valid_error = False
        else:
            self.valid_error = True
            self.error = msg.data

    def timer_callback(self):
        now = self.get_clock().now()
        dt = (now - self.prev_time).nanoseconds * 1e-9
        self.prev_time = now

        if not self.valid_error:
            self.pub.publish(Twist())
            self.get_logger().info('🛑 Sin referencia de línea – robot detenido')
            return

        if dt <= 0:
            return

        # Limitar el rango del error a un máximo razonable (por ejemplo, 150 px)
        max_error = 180.0
        self.error = np.clip(self.error, -max_error, max_error)

        # Control PD Adaptativo para el ángulo
        # Ganancia proporcional adaptativa (mayor para errores grandes, menor para errores pequeños)
        kp = self.kp_base * (1 + (abs(self.error) / max_error))

        derivative = (self.error - self.prev_error) / dt
        self.prev_error = self.error

        # Control Angular basado en el error y su derivada (AQUI CAMBIAMOS EL SIGNO)
        angular_z = -(kp * self.error + self.kd * derivative)

        # Control Lineal Proporcional
        max_speed = 0.6  # Velocidad máxima
        min_speed = 0.05  # Velocidad mínima
        # Disminuye la velocidad lineal suavemente si el error es grande
        linear_x = max(max_speed - abs(self.error) * 0.001, min_speed)

        # Limitar las velocidades para evitar movimientos bruscos
        angular_z = np.clip(angular_z, -1.2, 1.2)
        linear_x = np.clip(linear_x, min_speed, max_speed)

        # Publicar comando de velocidad
        cmd = Twist()
        cmd.linear.x = linear_x
        cmd.angular.z = angular_z
        self.pub.publish(cmd)

        self.get_logger().info(f'🚦 Control - Error: {self.error:.2f}, Angular: {angular_z:.3f}, Linear: {linear_x:.3f}')

def main(args=None):
    rclpy.init(args=args)
    node = LineController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
