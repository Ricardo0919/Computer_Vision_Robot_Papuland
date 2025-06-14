#!/usr/bin/env python3
# ------------------------------------------------------------------------------
# Proyecto: Puzzlebot Final Challenge - Nodo para tomar imágenes de señales de tráfico
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
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

# Cambia esto por tu clase actual ('stop', 'left', 'right', 'straight', 'worker', 'give_way', 'null')
CURRENT_CLASS = 'straight_extra'

# Carpeta de destino para guardar las imágenes
SAVE_PATH = f'/home/ricardosierra/Documents/TEC/6Semestre/ImplementacionDeRoboticaInteligente/ManchesterRobotics/Computer_Vision_Robot_Papuland/finalchallenge_ws/CNN/Signal/dataset/{CURRENT_CLASS}'
os.makedirs(SAVE_PATH, exist_ok=True)

class ImageSaver(Node):
    def __init__(self):
        super().__init__('image_saver')

        # Subscripción al topic de imagen
        self.subscription = self.create_subscription(
            Image,
            '/video_source/raw',
            self.listener_callback,
            10)
        self.bridge = CvBridge()
        self.image_count = 0

    # Muestra la imagen en una ventana y guarda si presionas 's'
    def listener_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv2.imshow('Captura', cv_image)
        key = cv2.waitKey(1)

        # Guardar imagen con 's'
        if key == ord('s'):
            filename = os.path.join(SAVE_PATH, f'img_{self.image_count:03d}.png')
            cv2.imwrite(filename, cv_image)
            print(f'[✔] Imagen guardada: {filename}')
            self.image_count += 1

        # Salir del nodo con 'q'
        if key == ord('q'):
            rclpy.shutdown()

# ---------------- main ----------------
def main(args=None):
    rclpy.init(args=args)
    node = ImageSaver()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
