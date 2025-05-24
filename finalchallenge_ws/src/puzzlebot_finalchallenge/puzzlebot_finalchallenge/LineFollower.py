#!/usr/bin/env python3
# ------------------------------------------------------------------------------
# Proyecto: Puzzlebot Final Challenge - Seguidor de línea
# Materia: Implementación de Robótica Inteligente
# Fecha: 12 de junio de 2025
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
from std_msgs.msg import Float32

class LineFollower(Node):
    def __init__(self):
        super().__init__('LineFollower')
        self.bridge = CvBridge()
        # Bandera para saber si ya se recibió una imagen
        self.image_received_flag = False

        # Declarar y obtener el parámetro de modo ('sim' para simulación o 'real' para físico)
        self.declare_parameter('mode', 'real')
        mode = self.get_parameter('mode').get_parameter_value().string_value

        self.declare_parameter('roi_ratio', 20)

        # Seleccionar el tópico de la cámara dependiendo del modo
        if mode == 'real':
            topic_name = 'video_source/raw'
        elif mode == 'sim':
            topic_name = 'camera'
        else:
            self.get_logger().warn(f'Modo "{mode}" no reconocido. Usando "sim" por defecto.')
            topic_name = 'camera'

        # Suscripción a la imagen de la cámara
        self.sub = self.create_subscription(Image, topic_name, self.camera_callback, 10)
        
        # Publicadores:
        # Publica el error de posición respecto al centro de la imagen
        self.pub_error = self.create_publisher(Float32, '/line_follower_data', 10)
        # Publica la imagen procesada con anotaciones
        self.pub_img = self.create_publisher(Image, '/processed_line_image', 10)

        # Timer para procesar imágenes a 10 Hz
        dt = 0.1
        self.timer = self.create_timer(dt, self.timer_callback)

        self.get_logger().info(f'LineFollower Node iniciado en modo: {mode}')

    def camera_callback(self, msg):
        # Convierte el mensaje ROS (sensor_msgs/Image) a imagen OpenCV (BGR)
        try:
            self.cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.image_received_flag = True
        except:
            self.get_logger().warning('No se pudo obtener una imagen!!!')

    def timer_callback(self):
        # Solo procesa si ya se recibió una imagen
        if self.image_received_flag:
            self.process_image()

    def process_image(self):
        # 1. Preprocesamiento
        gray_img = cv2.cvtColor(self.cv_img, cv2.COLOR_BGR2GRAY)
        thresh_value = 95  # Puedes ajustar este valor si lo haces parámetro
        _, mask = cv2.threshold(gray_img, thresh_value, 255, cv2.THRESH_BINARY_INV)
            
        # Operaciones morfológicas
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        
        # Definir ROI (últimos 100 píxeles)
        height, width = mask.shape
        sensor_height = height - self.get_parameter('roi_ratio').value
        roi_mask = mask[sensor_height:, :]
        
        # Detectar contornos en ROI
        contours, _ = cv2.findContours(roi_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Ajustar coordenadas
        adjusted_contours = []
        for contour in contours:
            adjusted = contour.copy()
            adjusted[:, :, 1] += sensor_height  # Ajustar coordenada Y
            adjusted_contours.append(adjusted)
            
        # Procesar y calcular error
        output_img = self.cv_img.copy()
        error = np.nan
        cx = cy = 0
        
        if adjusted_contours:
            # Seleccionar el contorno más grande
            largest_contour = max(adjusted_contours, key=cv2.contourArea)
            
            # Calcular momentos
            M = cv2.moments(largest_contour)
            if M["m00"] > 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                
                # Calcular error respecto al centro
                image_center = width // 2
                error = float(cx - image_center)
                
                # Dibujar elementos
                cv2.drawContours(output_img, [largest_contour], -1, (0, 255, 0), 1)
                cv2.circle(output_img, (cx, cy), 5, (0, 0, 255), -1)
                cv2.line(output_img, (image_center, 0), (image_center, height), (255, 0, 0), 1)
                cv2.line(output_img, (cx, cy), (image_center, cy), (0, 0, 255), 1)

        # Anotaciones visuales
        cv2.rectangle(output_img, (0, sensor_height), (width, height), (255, 0, 0), 2)
        cv2.putText(output_img, f'Error: {error:.2f}', (10, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)

        # 7. Publicar resultados
        self.pub_error.publish(Float32(data=error))
        #self.get_logger().info(f'Error: {error:.2f}')
        self.pub_img.publish(self.bridge.cv2_to_imgmsg(output_img, 'bgr8'))

def main(args=None):
    rclpy.init(args=args)
    node = LineFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()