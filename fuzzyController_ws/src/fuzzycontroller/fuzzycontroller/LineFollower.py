#!/usr/bin/env python3
# ------------------------------------------------------------------------------
# Proyecto: Navegación de Línea con Control Difuso Basado en Centroide - Seguidor de línea
# Materia: Implementación de Robótica Inteligente
# Fecha: 6 de junio de 2025
# Alumnos:
#   - Jonathan Arles Guevara Molina  | A01710380
#   - Ezzat Alzahouri Campos         | A01710709
#   - José Ángel Huerta Ríos         | A01710607
#   - Ricardo Sierra Roa             | A01709887
# ------------------------------------------------------------------------------

# ----- Librerías -----
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Float32


# ----- Nodo principal para seguimiento de línea -----
class LineFollower(Node):
    def __init__(self):
        super().__init__('LineFollower') # Inicializa el nodo con nombre

        self.bridge = CvBridge()  # Puente para convertir imágenes ROS <-> OpenCV
        self.image_received_flag = False  # Bandera para saber si ya llegó una imagen

        # ----- Parámetros configurables desde launch o YAML -----
        self.declare_parameter('mode', 'real')           # Modo: 'real' o 'simulado'
        self.declare_parameter('roi_ratio', 15)          # Altura del ROI desde abajo
        self.declare_parameter('roi_left', 20)           # Margen izquierdo del ROI
        self.declare_parameter('roi_right', 20)          # Margen derecho del ROI

        mode = self.get_parameter('mode').get_parameter_value().string_value
        topic_name = 'video_source/raw' if mode == 'real' else 'camera' # Tópico de la cámara

        # ----- Suscripción a la imagen de la cámara -----
        self.sub = self.create_subscription(Image, topic_name, self.camera_callback, 10)

        # ----- Publicadores -----
        self.pub_error = self.create_publisher(Float32, '/line_follower_data', 10)
        self.pub_img = self.create_publisher(Image, '/processed_line_image', 10)

         # ----- Timer que ejecuta el procesamiento cada 0.1 s -----
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.get_logger().info(f'LineFollower Node iniciado en modo: {mode}')

     # ----- Callback de imagen -----
    def camera_callback(self, msg):
        try:
            self.cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8") # Convierte a imagen OpenCV
            self.image_received_flag = True # Marca que ya hay imagen lista
        except:
            self.get_logger().warning('No se pudo obtener una imagen!!!')

    # ----- Callback de timer -----
    def timer_callback(self):
        if self.image_received_flag:
            self.process_image() # Procesa la imagen si ya llegó

    # ----- Función para procesar imagen y detectar línea -----
    def process_image(self):
        # Redimensiona imagen a 160x120 para procesamiento más rápido
        self.cv_img = cv2.resize(self.cv_img, (160, 120))

        # Convierte a escala de grises y aplica umbral inverso (línea oscura)
        gray_img = cv2.cvtColor(self.cv_img, cv2.COLOR_BGR2GRAY)
        _, mask = cv2.threshold(gray_img, 95, 255, cv2.THRESH_BINARY_INV)

        # Operaciones morfológicas para limpiar el ruido
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        # Define región de interés (ROI)
        height, width = mask.shape
        roi_top = height - self.get_parameter('roi_ratio').value
        roi_left = self.get_parameter('roi_left').value
        roi_right = self.get_parameter('roi_right').value
        roi_mask = mask[roi_top:, roi_left:width - roi_right]

        # Detecta contornos dentro del ROI
        contours, _ = cv2.findContours(roi_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Ajusta coordenadas de los contornos al sistema global de la imagen
        adjusted_contours = []
        for contour in contours:
            adjusted = contour.copy()
            adjusted[:, :, 0] += roi_left  # Corrige eje X
            adjusted[:, :, 1] += roi_top   # Corrige eje Y
            adjusted_contours.append(adjusted)

        # Copia imagen original para dibujar resultados
        output_img = self.cv_img.copy()
        error = np.nan
        cx = cy = 0 # Coordenadas del centroide

        # Si hay contornos válidos, encuentra el más grande y calcula su centroide
        if adjusted_contours:
            largest_contour = max(adjusted_contours, key=cv2.contourArea)
            M = cv2.moments(largest_contour)
            if M["m00"] > 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                image_center = width // 2
                error = float(cx - image_center) # Error: distancia horizontal al centro

                # ----- Dibujos para visualización -----
                cv2.drawContours(output_img, [largest_contour], -1, (0, 255, 0), 1)
                cv2.circle(output_img, (cx, cy), 5, (0, 0, 255), -1)
                cv2.line(output_img, (image_center, 0), (image_center, height), (255, 0, 0), 1)
                cv2.line(output_img, (cx, cy), (image_center, cy), (0, 0, 255), 1)

        # Dibuja el ROI sobre la imagen original
        cv2.rectangle(output_img, (roi_left, roi_top), (width - roi_right, height), (255, 0, 0), 2)                  # Envía el error al tópico
        cv2.putText(output_img, f'Error: {error:.2f}', (10, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)     # Imagen con anotaciones

        # ----- Publicación de datos -----
        self.pub_error.publish(Float32(data=error))
        self.pub_img.publish(self.bridge.cv2_to_imgmsg(output_img, 'bgr8'))

# ----- Main -----
def main(args=None):
    rclpy.init(args=args)
    node = LineFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
