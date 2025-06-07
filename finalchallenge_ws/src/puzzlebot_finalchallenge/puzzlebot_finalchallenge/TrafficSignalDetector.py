#!/usr/bin/env python3
# ------------------------------------------------------------------------------
# Proyecto: Puzzlebot Final Challenge - Nodo ROS2 para detección de señales de tráfico con YOLOv8
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
from ultralytics import YOLO
import numpy as np
import cv2
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

class YOLOv8TrafficSignalDetector(Node):
    def __init__(self):
        super().__init__('yolov8_traffic_signal_detector')

        # Suscripción al tópico de imagen
        self.subscription = self.create_subscription(Image, 'video_source/raw', self.image_callback, 10)
        self.bridge = CvBridge()
        self.cv_img = None
        self.new_img = False

        # Publicadores
        self.signal_pub = self.create_publisher(String, 'signal_detector', 10)
        self.image_pub = self.create_publisher(Image, 'traffic_signal', 10)

        # Temporizador
        self.create_timer(0.1, self.timer_callback)

        # Cargar modelo YOLOv8
        model_path = Path(get_package_share_directory('puzzlebot_finalchallenge')) / 'models' / 'TrafficSignal.pt'
        self.model = YOLO(str(model_path))
        self.names = self.model.names
        self.prev_detected = 'none'

        # Publicar estado inicial
        self.signal_pub.publish(String(data='none'))
        self.get_logger().info('🚧 Nodo YOLOv8 de señales iniciado y modelo cargado correctamente.')

    def image_callback(self, msg):
        try:
            self.cv_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.new_img = True
        except Exception as e:
            self.get_logger().warn(f'Error al convertir la imagen: {e}')

    def timer_callback(self):
        if not self.new_img:
            return
        self.new_img = False

        img = cv2.resize(self.cv_img.copy(), (160, 120))

        try:
            results = self.model(img, imgsz=160, conf=0.75)
            annotated_img = img.copy()
            detected = None

            if results[0].boxes is not None and len(results[0].boxes):
                # Tomar solo la primera detección
                box = results[0].boxes[0]
                cls_id = int(box.cls[0])
                conf = float(box.conf[0])
                label = self.names[cls_id]
                detected = label

                # Dibujar caja y etiqueta
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cv2.rectangle(annotated_img, (x1, y1), (x2, y2), (255, 255, 0), 1)
                text = f"{label} {conf:.2f}"
                cv2.putText(annotated_img, text, (x1, y1 - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)

            # Publicar la señal solo si cambia
            if detected and detected != self.prev_detected:
                self.prev_detected = detected
                self.signal_pub.publish(String(data=detected))

            # Publicar imagen anotada
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(annotated_img, encoding='bgr8'))

        except Exception as e:
            self.get_logger().warn(f'Error durante la inferencia: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = YOLOv8TrafficSignalDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
