#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from puzzlebot_minichallenge2.msg import PathPose
import time
import math
import yaml
from ament_index_python.packages import get_package_share_directory
import os

class PathGenerator(Node):
    def __init__(self):
        super().__init__('path_generator')
        
        # Cargar parámetros desde YAML
        self.declare_parameter('config_file', 'path_config.yaml')
        config_file = self.get_parameter('config_file').value
        self.load_config(config_file)
        
        # Publicadores y suscriptores
        self.path_pub = self.create_publisher(PathPose, '/path_pose', 10)
        # self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pose_sub = self.create_subscription(Odometry, '/odom', self.pose_callback, 10)
        
        # Variables de estado
        self.current_pose = None
        self.current_target = 0
        self.path_start_time = time.time()
        self.path_completed = False
        
        # Temporizador para publicar puntos de ruta
        self.timer = self.create_timer(0.1, self.publish_path_point)
        
        self.get_logger().info('Path Generator iniciado y cargando configuración...')

    def load_config(self, filename):
        """Carga la configuración de ruta desde archivo YAML"""
        package_dir = get_package_share_directory('puzzlebot_minichallenge2')
        config_path = os.path.join(package_dir, 'config', filename)
        
        try:
            with open(config_path, 'r') as file:
                config = yaml.safe_load(file)
                
            self.path_type = config['path_type']
            self.waypoints = config['waypoints']
            self.max_linear_speed = config['max_linear_speed']
            self.max_angular_speed = config['max_angular_speed']
            
            # Calcular velocidades y tiempos si es necesario
            self.calculate_path_parameters()
            
            self.get_logger().info(f"Configuración cargada: {config}")
            
        except Exception as e:
            self.get_logger().error(f"Error cargando configuración: {str(e)}")
            raise

    def calculate_path_parameters(self):
        """Calcula velocidades y tiempos basados en la configuración"""
        if self.path_type == 'time_based':
            for wp in self.waypoints:
                if 'time' in wp:
                    distance = math.sqrt((wp['x'] - wp['prev_x'])**2 + 
                                        (wp['y'] - wp['prev_y'])**2)
                    wp['linear_velocity'] = distance / wp['time']
                    wp['angular_velocity'] = 0.0  # Se ajustará en giros
                    
        elif self.path_type == 'velocity_based':
            for i, wp in enumerate(self.waypoints):
                if i > 0:
                    prev_wp = self.waypoints[i-1]
                    distance = math.sqrt((wp['x'] - prev_wp['x'])**2 + 
                                        (wp['y'] - prev_wp['y'])**2)
                    wp['time'] = distance / wp['linear_velocity']
        
        # Verificar que los puntos sean alcanzables
        self.check_reachability()

    def check_reachability(self):
        """Verifica si todos los puntos son alcanzables con las limitaciones del robot"""
        for wp in self.waypoints:
            if 'linear_velocity' in wp and wp['linear_velocity'] > self.max_linear_speed:
                self.get_logger().warning(
                    f"Punto ({wp['x']}, {wp['y']}) requiere velocidad lineal {wp['linear_velocity']:.2f} m/s "
                    f"(máxima permitida: {self.max_linear_speed:.2f} m/s)")
                wp['is_reachable'] = False
            else:
                wp['is_reachable'] = True

    def pose_callback(self, msg):
        """Actualiza la pose actual del robot"""
        self.current_pose = msg.pose.pose

    def calculate_error(self, target_x, target_y):
        """Calcula el error de posición actual respecto al objetivo"""
        if self.current_pose is None:
            return float('inf')
            
        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y
        return math.sqrt((target_x - current_x)**2 + (target_y - current_y)**2)

    def publish_path_point(self):
        """Publica el siguiente punto de la ruta"""
        if self.path_completed or len(self.waypoints) == 0:
            return
            
        current_target = self.waypoints[self.current_target]
        path_pose = PathPose()
        
        # Configurar el mensaje PathPose
        path_pose.pose.position.x = current_target['x']
        path_pose.pose.position.y = current_target['y']
        
        if 'linear_velocity' in current_target:
            path_pose.linear_velocity = current_target['linear_velocity']
        if 'angular_velocity' in current_target:
            path_pose.angular_velocity = current_target['angular_velocity']
        if 'time' in current_target:
            path_pose.time_to_reach = current_target['time']
            
        path_pose.is_reachable = current_target['is_reachable']
        
        # Publicar el punto de ruta
        self.path_pub.publish(path_pose)
        
        # Verificar si hemos llegado al punto actual
        error = self.calculate_error(current_target['x'], current_target['y'])
        if error < 0.1:  # Umbral de 10 cm
            self.get_logger().info(f"Punto {self.current_target} alcanzado. Error: {error:.3f} m")
            self.current_target += 1
            
            if self.current_target >= len(self.waypoints):
                self.path_completed = True
                self.get_logger().info("¡Ruta completada!")
                # Detener el robot
                self.cmd_vel_pub.publish(Twist())

def main(args=None):
    rclpy.init(args=args)
    node = PathGenerator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()