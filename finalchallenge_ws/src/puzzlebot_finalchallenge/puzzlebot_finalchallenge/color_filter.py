#!/usr/bin/env python3
# ------------------------------------------------------------------------------
# Proyecto: Puzzlebot Final Challenge - Nodo para filtrar color con camara de puzzlebot
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
import json
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class ColorFilterNode(Node):
    def __init__(self):
        super().__init__('ColorFilterNode')
        self.bridge = CvBridge()
        self.image_received_flag = False
        self.cv_img = None

        # Parámetro de modo: sim o real
        self.declare_parameter('mode', 'real')
        mode = self.get_parameter('mode').get_parameter_value().string_value

        topic_name = 'video_source/raw' if mode == 'real' else 'camera'
        self.sub = self.create_subscription(Image, topic_name, self.camera_callback, 10)
        self.get_logger().info(f'🎨 ColorFilterNode iniciado en modo: {mode}')

        # Crear ventanas
        cv2.namedWindow("Trackbars")
        cv2.namedWindow("Original", cv2.WINDOW_NORMAL)
        cv2.namedWindow("Mascara", cv2.WINDOW_NORMAL)
        cv2.namedWindow("Filtrado", cv2.WINDOW_NORMAL)

        # Trackbars HSV
        cv2.createTrackbar("H min", "Trackbars", 0, 180, lambda x: None)
        cv2.createTrackbar("H max", "Trackbars", 180, 180, lambda x: None)
        cv2.createTrackbar("S min", "Trackbars", 0, 255, lambda x: None)
        cv2.createTrackbar("S max", "Trackbars", 255, 255, lambda x: None)
        cv2.createTrackbar("V min", "Trackbars", 0, 255, lambda x: None)
        cv2.createTrackbar("V max", "Trackbars", 255, 255, lambda x: None)

        # Callback de mouse
        cv2.setMouseCallback("Filtrado", self.mouse_callback)

        # Cargar configuración previa
        self.load_hsv_preset()

        # Timer para procesamiento periódico
        self.timer = self.create_timer(0.1, self.timer_callback)

    def camera_callback(self, msg):
        try:
            self.cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.image_received_flag = True
        except:
            self.get_logger().warning("⚠️ No se pudo convertir la imagen")

    def timer_callback(self):
        if not self.image_received_flag or self.cv_img is None:
            return

        img = self.cv_img.copy()
        hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # Leer trackbars
        h_min = cv2.getTrackbarPos("H min", "Trackbars")
        h_max = cv2.getTrackbarPos("H max", "Trackbars")
        s_min = cv2.getTrackbarPos("S min", "Trackbars")
        s_max = cv2.getTrackbarPos("S max", "Trackbars")
        v_min = cv2.getTrackbarPos("V min", "Trackbars")
        v_max = cv2.getTrackbarPos("V max", "Trackbars")

        # Crear máscara HSV
        lower = np.array([h_min, s_min, v_min])
        upper = np.array([h_max, s_max, v_max])
        mask = cv2.inRange(hsv_img, lower, upper)

        # Limpiar la máscara
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # Aplicar máscara a la imagen original
        result = cv2.bitwise_and(img, img, mask=mask)

        # Mostrar imágenes
        cv2.imshow("Original", img)
        cv2.imshow("Mascara", mask)
        cv2.imshow("Filtrado", result)

        # Guardar configuración al presionar 's'
        key = cv2.waitKey(1)
        if key == ord('s'):
            self.save_hsv_preset()
            cv2.imwrite("mascara_amarillo.png", mask)
            cv2.imwrite("filtrado_amarillo.png", result)
            print("🟨 Imágenes y configuración guardadas")

    def mouse_callback(self, event, x, y, flags, param=None):
        if event == cv2.EVENT_LBUTTONDOWN and self.cv_img is not None:
            hsv_img = cv2.cvtColor(self.cv_img, cv2.COLOR_BGR2HSV)
            if y < hsv_img.shape[0] and x < hsv_img.shape[1]:
                h, s, v = hsv_img[y, x]
                print(f"📍 HSV en ({x},{y}) → H: {h}, S: {s}, V: {v}")

                # Ajustar trackbars automáticamente
                delta_h, delta_s, delta_v = 10, 40, 40
                h_min = max(h - delta_h, 0)
                h_max = min(h + delta_h, 180)
                s_min = max(s - delta_s, 0)
                s_max = min(s + delta_s, 255)
                v_min = max(v - delta_v, 0)
                v_max = min(v + delta_v, 255)

                cv2.setTrackbarPos("H min", "Trackbars", h_min)
                cv2.setTrackbarPos("H max", "Trackbars", h_max)
                cv2.setTrackbarPos("S min", "Trackbars", s_min)
                cv2.setTrackbarPos("S max", "Trackbars", s_max)
                cv2.setTrackbarPos("V min", "Trackbars", v_min)
                cv2.setTrackbarPos("V max", "Trackbars", v_max)

                print("🎯 Trackbars ajustadas automáticamente")

    def save_hsv_preset(self):
        preset = {
            'H min': cv2.getTrackbarPos("H min", "Trackbars"),
            'H max': cv2.getTrackbarPos("H max", "Trackbars"),
            'S min': cv2.getTrackbarPos("S min", "Trackbars"),
            'S max': cv2.getTrackbarPos("S max", "Trackbars"),
            'V min': cv2.getTrackbarPos("V min", "Trackbars"),
            'V max': cv2.getTrackbarPos("V max", "Trackbars")
        }
        with open('yellow_preset.json', 'w') as f:
            json.dump(preset, f)

    def load_hsv_preset(self):
        try:
            with open('yellow_preset.json', 'r') as f:
                preset = json.load(f)
                for key, value in preset.items():
                    cv2.setTrackbarPos(key, "Trackbars", value)
            print("✅ Configuración HSV cargada")
        except:
            print("ℹ️ No se encontró configuración previa")

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
