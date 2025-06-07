#!/usr/bin/env python3
# ------------------------------------------------------------------------------
# Proyecto: Puzzlebot Final Challenge - Nodo de captura de imágenes del semáforo
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
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

# Diccionario de clases y sus carpetas
CLASSES = {
    'green_track':  {'key': ord('g'), 'count': 0},
    'red_track':    {'key': ord('r'), 'count': 0},
    'yellow_track': {'key': ord('y'), 'count': 0}
}

# Ruta base
BASE_PATH = '/home/ricardosierra/Documents/TEC/6Semestre/ImplementacionDeRoboticaInteligente/ManchesterRobotics/Computer_Vision_Robot_Papuland/finalchallenge_ws/CNN/TrafficLight/dataset'

# Asegúrate de que existan las carpetas
for class_name in CLASSES:
    full_path = os.path.join(BASE_PATH, class_name)
    os.makedirs(full_path, exist_ok=True)

class ImageSaver(Node):
    def __init__(self):
        super().__init__('image_saver')
        self.subscription = self.create_subscription(
            Image,
            '/video_source/raw',
            self.listener_callback,
            10)
        self.bridge = CvBridge()

    def listener_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv2.imshow('Captura', cv_image)
        key = cv2.waitKey(1)

        for class_name, data in CLASSES.items():
            if key == data['key']:
                count = data['count']
                filename = os.path.join(BASE_PATH, class_name, f'img_{count:03d}.png')
                cv2.imwrite(filename, cv_image)
                print(f'[✔] Imagen guardada: {filename}')
                CLASSES[class_name]['count'] += 1

        if key == ord('q'):
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = ImageSaver()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
