#!/usr/bin/env python3
# ------------------------------------------------------------------------------
# Proyecto: Mini Challenge 4 - Nodo de Odometría
# Materia: Implementación de Robótica Inteligente
# Fecha: 14 de mayo de 2025
# Alumnos:
#   - Jonathan Arles Guevara Molina  | A01710380
#   - Ezzat Alzahouri Campos         | A01710709
#   - José Ángel Huerta Ríos         | A01710607
#   - Ricardo Sierra Roa             | A01709887
# ------------------------------------------------------------------------------

import rclpy
import transforms3d
import numpy as np
import signal
import sys
from rclpy import qos
from rclpy.node import Node
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry

class OdometryNode(Node):
    def __init__(self):
        super().__init__('OdometryNode')

        # Estado inicial de la pose del robot (X, Y, orientación)
        self.X = 0.0
        self.Y = 0.0
        self.Th = 0.0  # Ángulo theta (yaw)

        # Parámetros físicos del robot
        self._l = 0.19   # Distancia entre ruedas (track width)
        self._r = 0.0505 # Radio de las ruedas

        # Declaración de parámetros configurables desde launch
        self.declare_parameter('angular_correction_factor', 0.90)
        self.declare_parameter('linear_correction_factor', 0.92)
        self._angular_correction_factor = self.get_parameter('angular_correction_factor').value
        self._linear_correction_factor = self.get_parameter('linear_correction_factor').value

        # Parámetros de muestreo y frecuencia de actualización
        self._sample_time = 0.01  # Intervalo mínimo entre actualizaciones
        self.rate = 200.0         # Frecuencia de publicación (Hz)

        # Variables internas
        self.first = True
        self.last_time = 0.0
        self.v_r = 0.0
        self.v_l = 0.0
        self.V = 0.0
        self.Omega = 0.0

        # Subscripciones a las velocidades de cada rueda
        self.sub_encR = self.create_subscription(Float32, 'VelocityEncR', self.encR_callback, qos.qos_profile_sensor_data)
        self.sub_encL = self.create_subscription(Float32, 'VelocityEncL', self.encL_callback, qos.qos_profile_sensor_data)

        # Publicador de odometría
        self.odom_pub = self.create_publisher(Odometry, 'odom', qos.qos_profile_sensor_data)

        # Timer periódico para actualizar odometría
        self.timer = self.create_timer(1.0 / self.rate, self.run)

        # Mensaje de odometría que se reutiliza
        self.odom_msg = Odometry()

        self.get_logger().info("🧭 Nodo OdometryNode iniciado.")

    def encR_callback(self, msg):
        # Conversión de velocidad angular a lineal para rueda derecha
        self.v_r = self._r * msg.data

    def encL_callback(self, msg):
        # Conversión de velocidad angular a lineal para rueda izquierda
        self.v_l = self._r * msg.data

    def run(self):
        # Inicializar tiempo en la primera ejecución
        if self.first:
            self.last_time = self.get_clock().now()
            self.first = False
            return

        # Calcular delta de tiempo desde la última actualización
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9

        # Verificar si ha pasado suficiente tiempo para actualizar
        if dt >= self._sample_time:
            # Cálculo de velocidades (modelo de robot diferencial)
            self.V = (self.v_r + self.v_l) / 2.0               # Velocidad lineal promedio
            self.Omega = (self.v_r - self.v_l) / self._l       # Velocidad angular

            # Integración para actualizar la pose (con corrección)
            self.Th += self.Omega * dt * self._angular_correction_factor
            self.X  += self.V * np.cos(self.Th) * dt * self._linear_correction_factor
            self.Y  += self.V * np.sin(self.Th) * dt * self._linear_correction_factor

            self.last_time = now
            self.publish_odometry()

    def publish_odometry(self):
        # Convertir ángulo theta a quaternion para orientación
        q = transforms3d.euler.euler2quat(0, 0, self.Th)

        # Llenar mensaje de odometría
        self.odom_msg.header.stamp = self.get_clock().now().to_msg()
        self.odom_msg.header.frame_id = 'odom'
        self.odom_msg.child_frame_id = 'base_footprint'

        self.odom_msg.pose.pose.position.x = self.X
        self.odom_msg.pose.pose.position.y = self.Y
        self.odom_msg.pose.pose.position.z = 0.0

        self.odom_msg.pose.pose.orientation.x = q[1]
        self.odom_msg.pose.pose.orientation.y = q[2]
        self.odom_msg.pose.pose.orientation.z = q[3]
        self.odom_msg.pose.pose.orientation.w = q[0]

        self.odom_msg.twist.twist.linear.x = self.V
        self.odom_msg.twist.twist.angular.z = self.Omega

        # Publicar mensaje
        self.odom_pub.publish(self.odom_msg)

    def stop_handler(self, signum, frame):
        # Manejo de interrupción Ctrl+C para apagar el nodo con mensaje
        self.get_logger().info("❌ Interrupción recibida. Deteniendo nodo...")
        raise SystemExit

def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()

    # Capturar señal de interrupción (Ctrl+C)
    signal.signal(signal.SIGINT, node.stop_handler)

    try:
        rclpy.spin(node)
    except SystemExit:
        node.get_logger().info('🛑 Apagado del nodo OdometryNode.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

