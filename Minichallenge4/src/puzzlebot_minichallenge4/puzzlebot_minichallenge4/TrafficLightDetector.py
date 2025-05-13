#!/usr/bin/env python3
# ------------------------------------------------------------------------------
# Proyecto: Mini Challenge 4 - Nodo de detección de colores de semáforo
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
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
import signal
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

class TrafficLightDetector(Node):
    def __init__(self):
        super().__init__('color_detector')

        # Configuración de QoS para publicacion de colores
        qos_profile_color = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE,
            depth=10
        )

        # Obtener parámetro 'mode' (sim o real)
        self.declare_parameter('mode', 'sim')  # Valor por defecto
        mode = self.get_parameter('mode').get_parameter_value().string_value

        self.bridge = CvBridge()
        self.prev_state = ""  # Guardar el último estado para evitar publicaciones innecesarias

        # Seleccionar el tópico de imagen dependiendo del modo
        if mode == 'real':
            topic_name = 'video_source/raw'
        elif mode == 'sim':
            topic_name = 'camera'
        else:
            self.get_logger().warn(f'Modo "{mode}" no reconocido. Usando "real" por defecto.')
            topic_name = 'video_source/raw'

        # Subscripción a cámara y publishers
        self.sub = self.create_subscription(Image, topic_name, self.camera_callback, 10)
        self.pub_img = self.create_publisher(Image, 'processed_img', 10)
        self.pub_color = self.create_publisher(String, 'color_detector', qos_profile_color)

        self.get_logger().info('🚦 Nodo TrafficLightDetector iniciado.')

        # Rango de colores en HSV para detectar cada luz del semáforo
        self.hsv_ranges = {
            "Rojo": [
                {"lower": np.array([0, 100, 100]), "upper": np.array([10, 255, 255])},
                {"lower": np.array([160, 100, 100]), "upper": np.array([179, 255, 255])}
            ],
            "Verde": [
                {"lower": np.array([40, 50, 50]), "upper": np.array([80, 255, 255])}
            ],
            "Amarillo": [
                {"lower": np.array([20, 100, 100]), "upper": np.array([30, 255, 255])}
            ]
        }

        # Bandera para asegurar que llegó una imagen
        self.image_received_flag = False 
        dt = 0.1 
        # Procesa imagen cada 10 Hz
        self.timer = self.create_timer(dt, self.timer_callback) 

    def camera_callback(self, msg):
        # Convertir el mensaje ROS a imagen OpenCV
        try:
            self.cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.image_received_flag = True         
        except:
            self.get_logger().info('⚠️ Failed to get an image')
    
    def timer_callback(self): 
        if self.image_received_flag: 
            self.process_image() 

    def process_image(self):
        # Redimensionar y convertir a HSV
        resized_image = cv2.resize(self.cv_img, (160, 120))
        hsv_img = cv2.cvtColor(resized_image, cv2.COLOR_BGR2HSV)
        output_img = resized_image.copy()
        detected_colors = []

        # Detectar colores presentes según rangos HSV
        for color_name, ranges in self.hsv_ranges.items():
            mask = None
            for range_item in ranges:
                temp_mask = cv2.inRange(hsv_img, range_item["lower"], range_item["upper"])
                if mask is None:
                    mask = temp_mask
                else:
                    mask = cv2.bitwise_or(mask, temp_mask)

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area > 600:
                    x, y, w, h = cv2.boundingRect(cnt)
                    cv2.rectangle(output_img, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    detected_colors.append(color_name)

        # Escribir los colores detectados en esquina superior izquierda
        y_offset = 20
        for color in detected_colors:
            cv2.putText(output_img, color, (5, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
            y_offset += 15

        # Determinar el estado del semáforo
        new_state = "none"
        if "Rojo" in detected_colors:
            new_state = "stop"
        elif "Amarillo" in detected_colors:
            new_state = "slow"
        elif "Verde" in detected_colors:
            new_state = "continue"

        # Publicar solo si hay un cambio real de estado
        if new_state != self.prev_state:
            self.prev_state = new_state
            color_msg = String()
            color_msg.data = new_state
            self.pub_color.publish(color_msg)
            self.get_logger().info(f"🎨 Semáforo detectado: {color_msg.data.upper()}")

        # Publicar imagen procesada para visualización
        self.pub_img.publish(self.bridge.cv2_to_imgmsg(output_img, 'bgr8'))

def main(args=None):
    rclpy.init(args=args)
    node = TrafficLightDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

