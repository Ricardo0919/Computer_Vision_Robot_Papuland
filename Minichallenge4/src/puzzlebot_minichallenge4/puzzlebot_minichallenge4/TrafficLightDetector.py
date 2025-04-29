"""
This program detects red, green and yellow colors and publishes the processed image and detected action.

Published topics:
    /processed_img [Image]
    /color_detector [String]

Subscribed topics:
    /camera [Image]
"""

import rclpy
from rclpy.node import Node

import cv2
import numpy as np

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String  # <--- IMPORTANTE


class CVExample(Node):
    def __init__(self):
        super().__init__('color_detector')

        self.bridge = CvBridge()

        #self.sub = self.create_subscription(Image, 'video_source/raw', self.camera_callback, 10)
        self.sub = self.create_subscription(Image, 'camera', self.camera_callback, 10)
        self.pub_img = self.create_publisher(Image, 'processed_img', 10)
        self.pub_color = self.create_publisher(String, 'color_detector', 10)  # <--- Nuevo Publisher

        self.image_received_flag = False
        dt = 0.1
        self.timer = self.create_timer(dt, self.timer_callback)

        self.get_logger().info('ros_color_tracker Node started')

        # HSV Color ranges
        self.hsv_ranges = {
            "Rojo": [
                {"lower": np.array([0, 100, 100]), "upper": np.array([10, 255, 255])},
                {"lower": np.array([160, 100, 100]), "upper": np.array([179, 255, 255])}
            ],
            "Verde": [
                {"lower": np.array([40, 50, 50]), "upper": np.array([80, 255, 255])}
            ],
            "Amarillo": [
                {"lower": np.array([20, 100, 100]), "upper": np.array([30, 255, 255])}
            ]
        }

    def camera_callback(self, msg):
        try:
            self.cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.image_received_flag = True
        except:
            self.get_logger().info('Failed to get an image')

    def timer_callback(self):
        if self.image_received_flag:
            self.image_received_flag = False

            resized_image = cv2.resize(self.cv_img, (160, 120))

            hsv_img = cv2.cvtColor(resized_image, cv2.COLOR_BGR2HSV)

            output_img = resized_image.copy()

            detected_colors = []

            for color_name, ranges in self.hsv_ranges.items():
                mask = None
                for range_item in ranges:
                    temp_mask = cv2.inRange(hsv_img, range_item["lower"], range_item["upper"])
                    if mask is None:
                        mask = temp_mask
                    else:
                        mask = cv2.bitwise_or(mask, temp_mask)

                contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                for cnt in contours:
                    area = cv2.contourArea(cnt)
                    if area > 300:
                        x, y, w, h = cv2.boundingRect(cnt)
                        cv2.rectangle(output_img, (x, y), (x + w, y + h), (0, 255, 0), 2)
                        detected_colors.append(color_name)

            # Escribir los colores detectados en esquina superior izquierda
            y_offset = 20
            for color in detected_colors:
                cv2.putText(output_img, color, (5, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
                y_offset += 15

            # Publicar imagen procesada
            self.pub_img.publish(self.bridge.cv2_to_imgmsg(output_img, 'bgr8'))

            # Publicar acción basada en el color
            color_msg = String()
            if "Rojo" in detected_colors:
                color_msg.data = "stop"
            elif "Amarillo" in detected_colors:
                color_msg.data = "slow"
            elif "Verde" in detected_colors:
                color_msg.data = "continue"
            else:
                color_msg.data = "none"  # Opcional: si no hay detección

            self.pub_color.publish(color_msg)


def main(args=None):
    rclpy.init(args=args)

    cv_e = CVExample()

    rclpy.spin(cv_e)

    cv_e.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
