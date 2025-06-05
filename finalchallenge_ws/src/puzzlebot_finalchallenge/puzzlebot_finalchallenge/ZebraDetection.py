#!/usr/bin/env python3
# ------------------------------------------------------------------------------
# Proyecto: Puzzlebot Final Challenge - Detector de Cebra
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
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ZebraDetector(Node):
    def __init__(self):
        super().__init__('ZebraDetector')
        self.bridge = CvBridge()
        self.image_received_flag = False

        # Parámetros configurables
        self.declare_parameter('mode', 'real')
        self.declare_parameter('roi_top', 45)
        self.declare_parameter('roi_bottom', 110)
        self.declare_parameter('roi_left', 20)
        self.declare_parameter('roi_right', 20)

        mode = self.get_parameter('mode').get_parameter_value().string_value
        topic_name = 'video_source/raw' if mode == 'real' else 'camera'

        # Suscripción a la cámara
        self.sub = self.create_subscription(Image, topic_name, self.camera_callback, 10)

        # Publicadores
        self.pub_zebra = self.create_publisher(Bool, '/zebra_detected', 10)
        self.pub_debug_img = self.create_publisher(Image, '/zebra_image', 10)

        # Timer para procesamiento
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info(f'ZebraDetector Node iniciado en modo: {mode}')

    def camera_callback(self, msg):
        try:
            self.cv_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.image_received_flag = True
        except:
            self.get_logger().error('No se pudo convertir la imagen.')

    def timer_callback(self):
        if self.image_received_flag:
            self.process_image()

    def process_image(self):
        img = cv2.resize(self.cv_img, (160, 120))
        output_img = img.copy()

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        _, binary = cv2.threshold(gray, 95, 255, cv2.THRESH_BINARY_INV)
        binary = cv2.erode(binary, None, iterations=1)
        binary = cv2.dilate(binary, None, iterations=2)

        # Obtener parámetros del ROI
        roi_top = self.get_parameter('roi_top').value
        roi_bottom = self.get_parameter('roi_bottom').value
        roi_left = self.get_parameter('roi_left').value
        roi_right = self.get_parameter('roi_right').value

        # Calcular coordenadas del ROI
        roi = binary[roi_top:roi_bottom, roi_left:160 - roi_right]

        # Detección en el ROI
        contours, _ = cv2.findContours(roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        valid_contours = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if 170 < area < 700:
                approx = cv2.approxPolyDP(cnt, 0.04 * cv2.arcLength(cnt, True), True)
                if len(approx) >= 4:
                    valid_contours.append(cnt)

        square_count = len(valid_contours)
        zebra_detected = square_count >= 2
        self.pub_zebra.publish(Bool(data=zebra_detected))
        #self.get_logger().info(f'Cebra detectada: {zebra_detected} (cuadros: {square_count})')

        # Solo dibujar contornos si sí es cebra
        if zebra_detected:
            for cnt in valid_contours:
                cnt[:, 0, 0] += roi_left
                cnt[:, 0, 1] += roi_top
                cv2.drawContours(output_img, [cnt], -1, (0, 255, 0), 2)

        # Dibujar el ROI completo
        cv2.rectangle(output_img, (roi_left, roi_top), (160 - roi_right, roi_bottom), (255, 0, 0), 2)

        # Mostrar mensaje dinámico
        text = "CEBRA DETECTADA" if zebra_detected else "NO CEBRA"
        color = (0, 255, 0) if zebra_detected else (0, 0, 255)
        cv2.putText(output_img, text, (5, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)

        # Publicar imagen debug
        self.pub_debug_img.publish(self.bridge.cv2_to_imgmsg(output_img, 'bgr8'))

def main(args=None):
    rclpy.init(args=args)
    node = ZebraDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
