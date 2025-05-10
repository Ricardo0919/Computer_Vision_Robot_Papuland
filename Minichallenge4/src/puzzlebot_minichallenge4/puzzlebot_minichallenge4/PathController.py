#!/usr/bin/env python3
# ------------------------------------------------------------------------------
# Proyecto: Mini Challenge 4 - Nodo de Seguimiento de Trayectoria (PathController)
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
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from custom_interfaces.msg import PathPose
from std_msgs.msg import String
from rclpy.qos import qos_profile_sensor_data
import numpy as np

class PathController(Node):
    def __init__(self):
        super().__init__('PathController')

        # Subscripción a los puntos de la trayectoria
        self.pose_sub = self.create_subscription(PathPose, '/goals', self.path_callback, 10)
        
        # Subscripción a la odometría del robot
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, qos_profile_sensor_data)

        # Subscripción al detector de colores
        self.color_sub = self.create_subscription(String, '/color_detector', self.color_callback, 10)
        
        # Publicador de comandos de velocidad
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Lista de puntos objetivo (trayectoria)
        self.points = []
        self.target_index = 0
        self.goal_threshold = 0.08  # Tolerancia para considerar un punto alcanzado
        self.finished = False

        # Parámetros PID para control lineal y angular
        self.kp_lin = 1.0
        self.kd_lin = 0.1
        self.kp_ang = 4.0
        self.kd_ang = 0.3

        # Variables de error anteriores (para derivadas)
        self.prev_lin_error = 0.0
        self.prev_ang_error = 0.0
        self.prev_time = self.get_clock().now()

        # Estado del semáforo (por defecto en "continue")
        self.traffic_light_state = "continue"

        self.get_logger().info("🧭 PathController PID activado")

    def color_callback(self, msg):
        # Método para recibir y actualizar el estado del semáforo
        action_detected = msg.data

        # Lógica de persistencia de estados:
        if action_detected == "stop":               #RED
            self.traffic_light_state = "stop"       # Prioridad máxima: rojo
        elif action_detected == "slow":             #YELLOW    
            if self.traffic_light_state != "stop":  
                self.traffic_light_state = "slow"   # Solo cambia si no está en stop
        elif action_detected == "continue":         #GREEN
            self.traffic_light_state = "continue"   # Solo verde puede desbloquear el estado de stop
        
        self.get_logger().info(f"🎨 Estado del semáforo actualizado: {self.traffic_light_state}")

    def path_callback(self, msg):
        #Recibe nuevos puntos de la trayectoria y los agrega a la lista de objetivos.
        x = msg.pose.position.x
        y = msg.pose.position.y
        self.points.append((x, y))
        self.get_logger().info(f"➕ Añadido punto: ({x:.2f}, {y:.2f})")

    def odom_callback(self, msg):
        #Calcula y publica el comando de control basado en la posición actual y el siguiente punto objetivo.
        if self.finished or self.target_index >= len(self.points):
            return

        # Posición y orientación actual del robot
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, theta = self.euler_from_quaternion(q.x, q.y, q.z, q.w)

        # Punto objetivo actual
        goal_x, goal_y = self.points[self.target_index]
        dx = goal_x - x
        dy = goal_y - y
        distance = np.hypot(dx, dy)
        target_theta = np.arctan2(dy, dx)
        ang_error = self.normalize_angle(target_theta - theta)

        # Cálculo de delta de tiempo
        now = self.get_clock().now()
        dt = (now - self.prev_time).nanoseconds * 1e-9
        self.prev_time = now

        # Control lineal (solo si el ángulo no es muy desviado)
        lin_error = distance if abs(ang_error) < 0.3 else 0.0
        der_lin = (lin_error - self.prev_lin_error) / dt if dt > 0 else 0.0
        self.prev_lin_error = lin_error
        v = self.kp_lin * lin_error + self.kd_lin * der_lin

        # Control angular
        der_ang = (ang_error - self.prev_ang_error) / dt if dt > 0 else 0.0
        self.prev_ang_error = ang_error
        w = self.kp_ang * ang_error + self.kd_ang * der_ang

        # Crear y limitar comando de velocidad
        cmd = Twist()
        cmd.linear.x = np.clip(v, -0.3, 0.3)
        cmd.angular.z = np.clip(w, -1.5, 1.5)

        # Verificar si se alcanzó el punto
        if distance < self.goal_threshold:
            self.get_logger().info(f"✅ Punto {self.target_index + 1} alcanzado")
            self.target_index += 1

            # Si era el último punto, detener robot
            if self.target_index >= len(self.points):
                self.finished = True
                self.get_logger().info("🎯 Ruta completada")
                self.cmd_pub.publish(Twist())
                return

        # Publicar comando de velocidad
        self.cmd_pub.publish(cmd)

    def normalize_angle(self, angle):
        #Normaliza un ángulo al rango [-pi, pi].
        return np.arctan2(np.sin(angle), np.cos(angle))

    def euler_from_quaternion(self, x, y, z, w):
        #Convierte un quaternion a ángulos de Euler (roll, pitch, yaw).
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
