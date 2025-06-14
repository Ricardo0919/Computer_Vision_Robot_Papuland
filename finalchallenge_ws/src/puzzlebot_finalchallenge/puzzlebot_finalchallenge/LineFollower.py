#!/usr/bin/env python3
# ------------------------------------------------------------------------------
# Proyecto: Puzzlebot Final Challenge - Seguidor de línea
# Materia: Implementación de Robótica Inteligente
# Fecha: 14 de junio de 2025
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
        self.image_received_flag = False

        # Declarar parámetros configurables del nodo
        self.declare_parameter('mode', 'real')
        self.declare_parameter('roi_ratio', 15)
        self.declare_parameter('roi_left', 0)
        self.declare_parameter('roi_right', 0)

        # Determinar el topic de la cámara según el modo de operación
        mode = self.get_parameter('mode').get_parameter_value().string_value
        topic_name = 'video_source/raw' if mode == 'real' else 'camera'

        # Subscripción a imagen de cámara
        self.sub = self.create_subscription(Image, topic_name, self.camera_callback, 10)

        # Publicadores de error y de imagen debug
        self.pub_error = self.create_publisher(Float32, '/line_follower_data', 10)
        self.pub_img = self.create_publisher(Image, '/line_follower', 10)

        # Timer para procesamiento periódico (10 Hz)
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info(f'LineFollower Node iniciado en modo: {mode}')

    # Callback para convertir imagen ROS a OpenCV
    def camera_callback(self, msg):
        try:
            self.cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.image_received_flag = True
        except:
            self.get_logger().warning('No se pudo obtener una imagen!!!')

    # Ejecuta procesamiento si hay imagen disponible
    def timer_callback(self):
        if self.image_received_flag:
            self.process_image()

    # Procesamiento principal de imagen para detección de línea y cálculo de error
    def process_image(self):
        self.cv_img = cv2.resize(self.cv_img, (160, 120))
        gray_img = cv2.cvtColor(self.cv_img, cv2.COLOR_BGR2GRAY)
        _, mask = cv2.threshold(gray_img, 95, 255, cv2.THRESH_BINARY_INV)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        # Aplicar ROI para filtrar la parte inferior de la imagen
        height, width = mask.shape
        roi_top = height - self.get_parameter('roi_ratio').value
        roi_left = self.get_parameter('roi_left').value
        roi_right = self.get_parameter('roi_right').value
        roi_mask = mask[roi_top:, roi_left:width - roi_right]

        # Buscar contornos en el ROI
        contours, _ = cv2.findContours(roi_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Ajustar coordenadas de los contornos al frame completo
        adjusted_contours = []
        for contour in contours:
            adjusted = contour.copy()
            adjusted[:, :, 0] += roi_left  # X
            adjusted[:, :, 1] += roi_top   # Y
            adjusted_contours.append(adjusted)

        output_img = self.cv_img.copy()
        error = np.nan  # valor por defecto si no se detecta línea
        cx = cy = 0

        # Cálculo del centroide del contorno más grande
        if adjusted_contours:
            largest_contour = max(adjusted_contours, key=cv2.contourArea)
            M = cv2.moments(largest_contour)
            if M["m00"] > 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                image_center = width // 2
                error = float(cx - image_center)

                # Dibujar información visual del error
                cv2.drawContours(output_img, [largest_contour], -1, (0, 255, 0), 1)
                cv2.circle(output_img, (cx, cy), 5, (0, 0, 255), -1)
                cv2.line(output_img, (image_center, 0), (image_center, height), (255, 0, 0), 1)
                cv2.line(output_img, (cx, cy), (image_center, cy), (0, 0, 255), 1)

        # Dibujar el rectángulo del ROI
        cv2.rectangle(output_img, (roi_left, roi_top), (width - roi_right, height), (255, 0, 0), 2)
        cv2.putText(output_img, f'Error: {error:.2f}', (10, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)

        # Publicar error e imagen debug
        self.pub_error.publish(Float32(data=error))
        self.pub_img.publish(self.bridge.cv2_to_imgmsg(output_img, 'bgr8'))

# ---------------- main ----------------
def main(args=None):
    rclpy.init(args=args)
    node = LineFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
