#!/usr/bin/env python3
# ------------------------------------------------------------------------------
# Proyecto: Final Challenge - Nodo de detección de luces del semáforo
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
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import onnxruntime as ort
import os
from ament_index_python.packages import get_package_share_directory

class TrafficLightDetector(Node):
    def __init__(self):
        super().__init__('TrafficLightDetector')

        # Ruta al modelo ONNX
        package_path = get_package_share_directory('puzzlebot_finalchallenge')
        self.model_path = os.path.join(package_path, 'models', 'traffic_light_model.onnx')

        # Cargar modelo ONNX
        self.session = ort.InferenceSession(self.model_path, providers=["CPUExecutionProvider"])
        self.input_name = self.session.get_inputs()[0].name

        # Herramientas de ROS
        self.bridge = CvBridge()
        self.image_received_flag = False
        self.cv_img = None
        self.last_color = None  # Para evitar publicaciones repetidas

        # Suscripción a la cámara
        self.subscription = self.create_subscription(
            Image,
            '/video_source/raw',
            self.camera_callback,
            10
        )

        # Publicadores
        self.publisher = self.create_publisher(String, '/color_detector', 10)
        # Timer para procesar la imagen a 10 Hz
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info("🚦 Nodo de clasificación de semáforo iniciado.")

    def camera_callback(self, msg):
        try:
            self.cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.image_received_flag = True
        except Exception as e:
            self.get_logger().error(f"Error al convertir imagen: {e}")

    def timer_callback(self):
        if self.image_received_flag:
            self.process_image()

    def preprocess(self, cv_image):
        img = cv2.resize(cv_image, (160, 120))
        img = img.astype(np.float32) / 255.0
        img = np.expand_dims(img, axis=0)
        return img

    def process_image(self):
        try:
            input_tensor = self.preprocess(self.cv_img)
            output = self.session.run(None, {self.input_name: input_tensor})[0]
            pred_class = np.argmax(output)

            # Mapeo de clases
            class_map = ['green', 'off', 'red', 'yellow']
            color_detected = class_map[pred_class]

            # Solo publicar si cambió
            if color_detected != self.last_color:
                msg_out = String()
                msg_out.data = color_detected
                self.publisher.publish(msg_out)
                self.get_logger().info(f"Color detectado: {color_detected}")
                self.last_color = color_detected

        except Exception as e:
            self.get_logger().error(f"Error al procesar imagen: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = TrafficLightDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
