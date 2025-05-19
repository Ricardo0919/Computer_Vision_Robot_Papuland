#!/usr/bin/env python3
# ------------------------------------------------------------------------------
# Proyecto: Mini Challenge 5 - Nodo de detección de línea (3 líneas robustas con filtro de negro)
# Materia: Implementación de Robótica Inteligente
# Fecha: 21 de mayo de 2025
# Alumno:
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
        super().__init__('line_follower')
        self.bridge = CvBridge()
        self.image_received_flag = False

        # Declarar y obtener parámetro de modo (sim o real)
        self.declare_parameter('mode', 'sim')
        mode = self.get_parameter('mode').get_parameter_value().string_value

        # Elegir el tópico en función del modo
        if mode == 'real':
            topic_name = 'video_source/raw'
        elif mode == 'sim':
            topic_name = 'camera'
        else:
            self.get_logger().warn(f'Modo "{mode}" no reconocido. Usando "sim" por defecto.')
            topic_name = 'camera'

        # Suscripción a la cámara
        self.sub = self.create_subscription(Image, topic_name, self.camera_callback, 10)
        
        # Publicadores
        self.pub_error = self.create_publisher(Float32, '/line_follower_data', 10)
        self.pub_img = self.create_publisher(Image, '/processed_line_image', 10)

        # Timer para procesamiento
        dt = 0.1  # Frecuencia de procesamiento (10 Hz)
        self.timer = self.create_timer(dt, self.timer_callback)

        self.get_logger().info(f'📸 LineFollower Node iniciado en modo: {mode}')

    def camera_callback(self, msg):
        try:
            self.cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.image_received_flag = True
        except:
            self.get_logger().warning('⚠️ No se pudo obtener una imagen')

    def timer_callback(self):
        if self.image_received_flag:
            self.process_image()

    def process_image(self):
        # Convertir a HSV y aplicar máscara para negro
        hsv_img = cv2.cvtColor(self.cv_img, cv2.COLOR_BGR2HSV)
        lower_black = np.array([0, 0, 0])
        upper_black = np.array([180, 255, 50])
        mask = cv2.inRange(hsv_img, lower_black, upper_black)

        # Aplicar morfología para limpiar la máscara
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        # Encontrar contornos de las líneas negras
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        output_img = self.cv_img.copy()  # Imagen para visualización

        # Filtrar contornos pequeños para evitar ruido
        contours = [cnt for cnt in contours if cv2.contourArea(cnt) > 500]

        # Ordenar contornos por área (de mayor a menor)
        contours = sorted(contours, key=cv2.contourArea, reverse=True)

        line_centers = []

        # Detectar y calcular los centros de las tres líneas más grandes
        for contour in contours[:3]:  # Limitar a las tres más grandes
            M = cv2.moments(contour)
            if M["m00"] > 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                line_centers.append((cx, cy))
                cv2.drawContours(output_img, [contour], -1, (0, 255, 0), 2)  # Verde para contornos

        # Ordenar los centros de las líneas por su posición en X (de izquierda a derecha)
        line_centers.sort(key=lambda x: x[0])
        image_center = self.cv_img.shape[1] // 2

        # Selección de la línea central
        if len(line_centers) == 3:
            cx, cy = line_centers[1]  # Siempre la del medio
        
        elif len(line_centers) == 2:
            # De las dos, elige la que está más cerca de la cámara
            # (la que tiene el centroide con mayor valor en Y)
            cx, cy = max(line_centers, key=lambda pt: pt[1])

        else:
            self.get_logger().warning('⚠️ No se detectaron suficientes líneas para calcular el error.')
            self.pub_error.publish(Float32(data=np.nan))
            self.pub_img.publish(self.bridge.cv2_to_imgmsg(output_img, 'bgr8'))
            return

        # Calcular el error basado en el centro de la línea seleccionada
        error = float(cx - image_center)
        self.pub_error.publish(Float32(data=error))

        # Visualización (dibujo en imagen)
        cv2.line(output_img, (image_center, 0), (image_center, self.cv_img.shape[0]), (0, 255, 255), 2)  # Línea amarilla (centro)
        cv2.circle(output_img, (cx, cy), 7, (0, 0, 255), -1)  # Punto rojo (centro de la línea central)
        cv2.line(output_img, (cx, cy), (image_center, cy), (0, 255, 255), 2)  # Línea amarilla (error)
        cv2.putText(output_img, f'Error: {error:.2f}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        self.get_logger().info(f'📐 Error de línea central: {error}')

        # Publicar la imagen procesada
        self.pub_img.publish(self.bridge.cv2_to_imgmsg(output_img, 'bgr8'))

def main(args=None):
    rclpy.init(args=args)
    node = LineFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
