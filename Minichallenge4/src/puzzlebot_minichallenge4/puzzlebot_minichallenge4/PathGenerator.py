#!/usr/bin/env python3
# ------------------------------------------------------------------------------
# Proyecto: Mini Challenge 4 - Nodo Generador de Trayectorias
# Materia: Implementación de Robótica Inteligente
# Fecha: 14 de mayo de 2025
# Alumnos:
#   - Jonathan Arles Guevara Molina  | A01710380
#   - Ezzat Alzahouri Campos         | A01710709
#   - José Ángel Huerta Ríos         | A01710607
#   - Ricardo Sierra Roa             | A01709887
# ------------------------------------------------------------------------------

import rclpy
from rclpy.node import Node
from custom_interfaces.msg import PathPose
from geometry_msgs.msg import Point, Quaternion
import os
import yaml
import numpy as np
import signal
import sys


class PathGenerator(Node):
    def __init__(self):
        super().__init__('PathGenerator')

        # Declarar y leer parámetro con la ruta del archivo de trayectoria
        self.declare_parameter('path_file', '')
        path_file = self.get_parameter('path_file').get_parameter_value().string_value

        # Verificar existencia del archivo
        if not path_file or not os.path.exists(path_file):
            self.get_logger().error(f"❌ Archivo 'path.yaml' no encontrado en la ruta: {path_file}")
            return

        # Cargar contenido del archivo YAML
        with open(path_file, 'r') as f:
            config = yaml.safe_load(f)

        # Leer los waypoints
        self.waypoints = config.get('waypoints', [])

        # Parámetros físicos del robot (límite de velocidades)
        self.max_linear_speed = 2.0   # m/s
        self.max_angular_speed = 6.0  # rad/s

        # Validar que el primer punto sea el origen
        if not self.waypoints or (self.waypoints[0]['x'] != 0.0 or self.waypoints[0]['y'] != 0.0):
            self.get_logger().info("Insertando (0,0) al inicio por defecto.")
            self.waypoints.insert(0, {'x': 0.0, 'y': 0.0})

        # Crear publicador
        self.publisher_ = self.create_publisher(PathPose, '/goals', 10)

        # Validar la trayectoria y publicar si es válida
        self.validate_and_publish()

    def validate_and_publish(self):
        #Valida cada segmento de la trayectoria con base en las capacidades físicas del robot.
        #Publica solo si todos los puntos son alcanzables.
        
        all_reachable = True
        msgs = []

        for i, wp in enumerate(self.waypoints):
            # Crear mensaje para cada punto
            msg = PathPose()
            msg.pose.position = Point(x=wp['x'], y=wp['y'], z=0.0)
            msg.pose.orientation = Quaternion(w=1.0)  # Se asume sin rotación
            msg.is_reachable = True  # Por defecto, el punto es alcanzable

            # Validaciones a partir del segundo punto
            if i > 0:
                p1 = self.waypoints[i - 1]
                p2 = wp

                dx = p2['x'] - p1['x']
                dy = p2['y'] - p1['y']
                distance = np.hypot(dx, dy)
                angle = np.arctan2(dy, dx)

                # Validación de velocidad angular (solo si hay dos giros previos)
                if i > 1:
                    p0 = self.waypoints[i - 2]
                    prev_angle = np.arctan2(p1['y'] - p0['y'], p1['x'] - p0['x'])
                    dtheta = abs(self.normalize_angle(angle - prev_angle))
                    required_angular_speed = dtheta / 1.0  # Se asume que el giro debe hacerse en 1s

                    if required_angular_speed > self.max_angular_speed:
                        msg.is_reachable = False
                        self.get_logger().warn(
                            f"⚠️ Punto {i+1}: giro muy brusco ({np.degrees(dtheta):.2f}°) > máx. vel. angular"
                        )

                # Validación de velocidad lineal
                required_linear_speed = distance / 1.0  # Se asume 1 segundo por segmento
                if required_linear_speed > self.max_linear_speed:
                    msg.is_reachable = False
                    self.get_logger().warn(
                        f"⚠️ Punto {i+1}: requiere {required_linear_speed:.2f} m/s > máx. {self.max_linear_speed:.2f} m/s"
                    )

            estado = "✅ alcanzable" if msg.is_reachable else "❌ NO alcanzable"
            self.get_logger().info(f"Punto {i+1}: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f}) → {estado}")

            if not msg.is_reachable:
                all_reachable = False

            msgs.append(msg)

        # Publicar los puntos si todos son alcanzables
        if all_reachable:
            self.get_logger().info("✅ Todos los puntos son alcanzables. Publicando trayectoria...")
            for msg in msgs:
                self.publisher_.publish(msg)
        else:
            self.get_logger().error("❌ Algunos puntos no son alcanzables. No se publica la trayectoria.")

    def normalize_angle(self, angle):
        #Normaliza un ángulo al rango [-pi, pi].
        return np.arctan2(np.sin(angle), np.cos(angle))

def main(args=None):
    rclpy.init(args=args)
    node = PathGenerator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()