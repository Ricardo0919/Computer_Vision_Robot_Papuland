#!/usr/bin/env python3
# ------------------------------------------------------------------------------
# Nodo de realce con prioridad en amarillo
# ------------------------------------------------------------------------------

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rcl_interfaces.msg import SetParametersResult
import cv2
import numpy as np

class EnhancedTrafficLight(Node):
    def __init__(self):
        super().__init__('enhanced_traffic_light')
        
        # Parámetros dinámicos (valores por defecto enfocados en amarillo)
        self.declare_parameters(namespace='',
            parameters=[
                ('brightness_reduction', 0.7),
                ('global_sat', 1.2),
                ('red_boost', 1.8),
                ('green_boost', 1.6),
                ('yellow_boost', 3.0),  # Valor más alto para amarillo
                ('yellow_hue', [20, 30])  # Rango de tonos amarillos
            ])
        
        self._load_params()
        self.bridge = CvBridge()
        self.kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        
        # Configuración de tópicos
        self.subscription = self.create_subscription(
            Image,
            '/video_source/raw',
            self.process_image,
            10)
            
        self.publisher = self.create_publisher(
            Image,
            '/enhanced_image',
            10)

    def _load_params(self):
        params = [
            'brightness_reduction', 'global_sat',
            'red_boost', 'green_boost', 'yellow_boost', 'yellow_hue'
        ]
        for param in params:
            setattr(self, param, self.get_parameter(param).value)
        self.add_on_set_parameters_callback(self.param_callback)

    def param_callback(self, params):
        for param in params:
            value = param.value
            if param.name == 'yellow_hue':
                value = [int(v) for v in value]
            setattr(self, param.name, value)
            self.get_logger().info(f'Actualizado {param.name}: {value}')
        return SetParametersResult(successful=True)

    def process_image(self, msg):
        try:
            # 1. Reducción de brillo en YUV
            img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            yuv = cv2.cvtColor(img, cv2.COLOR_BGR2YUV)
            yuv[:,:,0] = np.clip(yuv[:,:,0] * self.brightness_reduction, 0, 255).astype(np.uint8)
            
            # 2. Conversión a HSV y ajustes
            hsv = cv2.cvtColor(cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR), cv2.COLOR_BGR2HSV)
            hsv[:,:,1] = np.clip(hsv[:,:,1] * self.global_sat, 0, 255).astype(np.uint8)
            
            # 3. Realce específico (amarillo primero)
            yellow_mask = self.create_yellow_mask(hsv)
            hsv[:,:,1] = np.where(yellow_mask,
                np.clip(hsv[:,:,1] * self.yellow_boost, 0, 255),
                hsv[:,:,1]
            ).astype(np.uint8)
            
            # 4. Realce secundario (rojo y verde)
            red_mask = self.create_red_mask(hsv)
            green_mask = self.create_green_mask(hsv)
            hsv[:,:,1] = np.where(red_mask, 
                np.clip(hsv[:,:,1] * self.red_boost, 0, 255), 
                hsv[:,:,1]).astype(np.uint8)
            hsv[:,:,1] = np.where(green_mask, 
                np.clip(hsv[:,:,1] * self.green_boost, 0, 255), 
                hsv[:,:,1]).astype(np.uint8)
            
            # 5. Publicar resultado
            enhanced_img = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
            self.publisher.publish(self.bridge.cv2_to_imgmsg(enhanced_img, "bgr8"))

        except Exception as e:
            self.get_logger().error(f'Error: {str(e)}')

    def create_yellow_mask(self, hsv):
        lower = np.array([self.yellow_hue[0], 50, 50])
        upper = np.array([self.yellow_hue[1], 255, 255])
        mask = cv2.inRange(hsv, lower, upper)
        return cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)

    def create_red_mask(self, hsv):
        lower1 = np.array([0, 50, 50])
        upper1 = np.array([10, 255, 255])
        lower2 = np.array([160, 50, 50])
        upper2 = np.array([179, 255, 255])
        mask = cv2.bitwise_or(
            cv2.inRange(hsv, lower1, upper1),
            cv2.inRange(hsv, lower2, upper2)
        )
        return cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)

    def create_green_mask(self, hsv):
        lower = np.array([40, 50, 50])
        upper = np.array([90, 255, 255])
        mask = cv2.inRange(hsv, lower, upper)
        return cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)

def main(args=None):
    rclpy.init(args=args)
    node = EnhancedTrafficLight()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()