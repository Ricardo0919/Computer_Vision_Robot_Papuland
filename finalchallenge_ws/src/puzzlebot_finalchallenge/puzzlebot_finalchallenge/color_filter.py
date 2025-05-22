#!/usr/bin/env python3
# ------------------------------------------------------------------------------
# Proyecto: Nodo para filtrar colores con trackbars HSV
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

class ColorFilterNode(Node):
    def __init__(self):
        super().__init__('ColorFilterNode')
        self.bridge = CvBridge()
        self.image_received_flag = False

        # Parámetro de modo: sim o real
        self.declare_parameter('mode', 'real')
        mode = self.get_parameter('mode').get_parameter_value().string_value

        if mode == 'real':
            topic_name = 'video_source/raw'
        elif mode == 'sim':
            topic_name = 'camera'
        else:
            self.get_logger().warn(f'Modo "{mode}" no reconocido. Usando "sim" por defecto.')
            topic_name = 'camera'

        self.sub = self.create_subscription(Image, topic_name, self.camera_callback, 10)
        self.get_logger().info(f'🎨 ColorFilterNode iniciado en modo: {mode}')

        # Crear ventana de trackbars
        cv2.namedWindow("Trackbars")
        cv2.createTrackbar("H min", "Trackbars", 0, 180, lambda x: None)
        cv2.createTrackbar("H max", "Trackbars", 180, 180, lambda x: None)
        cv2.createTrackbar("S min", "Trackbars", 0, 255, lambda x: None)
        cv2.createTrackbar("S max", "Trackbars", 255, 255, lambda x: None)
        cv2.createTrackbar("V min", "Trackbars", 0, 255, lambda x: None)
        cv2.createTrackbar("V max", "Trackbars", 255, 255, lambda x: None)

        # Crear ventanas redimensionables
        cv2.namedWindow("Original", cv2.WINDOW_NORMAL)
        cv2.namedWindow("Mascara", cv2.WINDOW_NORMAL)
        cv2.namedWindow("Filtrado", cv2.WINDOW_NORMAL)

        # Timer para procesar imagen
        self.timer = self.create_timer(0.1, self.timer_callback)

    def camera_callback(self, msg):
        try:
            self.cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.image_received_flag = True
        except:
            self.get_logger().warning("⚠️ No se pudo convertir la imagen")

    def timer_callback(self):
        if not self.image_received_flag:
            return

        img = self.cv_img.copy()
        hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # Leer valores HSV desde las trackbars
        h_min = cv2.getTrackbarPos("H min", "Trackbars")
        h_max = cv2.getTrackbarPos("H max", "Trackbars")
        s_min = cv2.getTrackbarPos("S min", "Trackbars")
        s_max = cv2.getTrackbarPos("S max", "Trackbars")
        v_min = cv2.getTrackbarPos("V min", "Trackbars")
        v_max = cv2.getTrackbarPos("V max", "Trackbars")

        lower_hsv = np.array([h_min, s_min, v_min])
        upper_hsv = np.array([h_max, s_max, v_max])
        mask = cv2.inRange(hsv_img, lower_hsv, upper_hsv)

        result = cv2.bitwise_and(img, img, mask=mask)

        # Redimensionar imágenes a 640x480
        resized_img = cv2.resize(img, (640, 480))
        resized_mask = cv2.resize(mask, (640, 480))
        resized_result = cv2.resize(result, (640, 480))

        # Mostrar imágenes
        cv2.imshow("Original", resized_img)
        cv2.imshow("Mascara", resized_mask)
        cv2.imshow("Filtrado", resized_result)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = ColorFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()
