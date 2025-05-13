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

class PathGenerator(Node):
    def __init__(self):
        super().__init__('PathGenerator')

        # Declarar y obtener el parámetro 'path_file' desde un launch file o línea de comandos
        self.declare_parameter('path_file', '')
        path_file = self.get_parameter('path_file').get_parameter_value().string_value

        # Verificar si el archivo existe en la ruta proporcionada
        if not path_file or not os.path.exists(path_file):
            self.get_logger().error(f"❌ Archivo 'path.yaml' no encontrado en la ruta: {path_file}")
            return

        # Cargar contenido del archivo YAML
        with open(path_file, 'r') as f:
            config = yaml.safe_load(f)

        # Leer los waypoints
        self.waypoints = config.get('waypoints', [])
        if not self.waypoints:
            self.get_logger().error("❌ No se encontraron puntos en el archivo de trayectoria.")
            return

        # Parámetros físicos del robot (límite de velocidades)
        self.max_linear_speed = 0.8   # Velocidad lineal máxima en m/s
        self.max_angular_speed = 6.0  # Velocidad angular máxima en rad/s

        # Validar que el primer punto sea el origen
        if not self.waypoints or (self.waypoints[0]['x'] != 0.0 or self.waypoints[0]['y'] != 0.0):
            self.get_logger().info("Insertando (0,0) al inicio por defecto.")
            self.waypoints.insert(0, {'x': 0.0, 'y': 0.0})

        # Crear publicador
        self.publisher_ = self.create_publisher(PathPose, '/goals', 10)

        # Validar la trayectoria y configurar temporizador para publicar
        self.validate_trajectory()
        self.get_logger().info("✅ Publicando puntos de trayectoria cada 1 segundo...")
        self.timer = self.create_timer(1.0, self.publish_points)
    
    def validate_trajectory(self):
        # Valida cada segmento de la trayectoria con base en las capacidades físicas del robot.
        all_reachable = True
        self.validated_points = []

        for i, wp in enumerate(self.waypoints):
            # Crear mensaje para cada punto
            msg = PathPose()
            msg.pose.position = Point(x=wp['x'], y=wp['y'], z=0.0)
            msg.pose.orientation = Quaternion(w=1.0)  # Sin orientación (plano XY)
            msg.is_reachable = True  # Por defecto, el punto es alcanzable

            # A partir del segundo punto, se validan velocidades requeridas
            if i > 0:
                p1 = self.waypoints[i - 1]
                p2 = wp

                dx = p2['x'] - p1['x']
                dy = p2['y'] - p1['y']
                distance = np.hypot(dx, dy) # Distancia euclidiana
                angle = np.arctan2(dy, dx)  # Dirección del movimiento

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

                # Validar velocidad lineal requerida entre dos puntos
                required_linear_speed = distance / 1.0  # Se asume 1 segundo por segmento
                if required_linear_speed > self.max_linear_speed:
                    msg.is_reachable = False
                    self.get_logger().warn(
                        f"⚠️ Punto {i+1}: requiere {required_linear_speed:.2f} m/s > máx. {self.max_linear_speed:.2f} m/s"
                    )

            # Mostrar estado del punto (alcanzable o no)
            estado = "✅ alcanzable" if msg.is_reachable else "❌ NO alcanzable"
            self.get_logger().info(f"Punto {i+1}: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f}) → {estado}")

            if not msg.is_reachable:
                all_reachable = False

            # Agregar punto a la lista final
            self.validated_points.append(msg)

        if not all_reachable:
            self.get_logger().error("❌ Algunos puntos no son alcanzables. No se publicarán.")

    def publish_points(self):
        # Publica los puntos validados uno por uno en el tópico '/goals', solo se publican puntos alcanzables.
        if not self.validated_points:
            self.get_logger().error("❌ No hay puntos validados para publicar.")
            return

        self.get_logger().info("📍 Publicando puntos de trayectoria...")
        for msg in self.validated_points:
            if msg.is_reachable:
                self.publisher_.publish(msg)
                self.get_logger().info(f"📍 Publicado punto: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})")

        self.get_logger().info("✅ Todos los puntos publicados. Deteniendo publicación.")
        self.timer.cancel()  # Detener el temporizador para evitar publicar más puntos.


    def normalize_angle(self, angle):
        # Normaliza un ángulo al rango [-pi, pi].
        return np.arctan2(np.sin(angle), np.cos(angle))


def main(args=None):
    rclpy.init(args=args)
    node = PathGenerator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
