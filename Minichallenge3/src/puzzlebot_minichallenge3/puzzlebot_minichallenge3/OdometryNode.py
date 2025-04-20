#!/usr/bin/env python3
import rclpy
import transforms3d
import numpy as np
import signal
from rclpy import qos
from rclpy.node import Node
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry


class OdometryNode(Node):

    def __init__(self):
        super().__init__('OdometryNode')

        # Estado inicial
        self.X = 0.0
        self.Y = 0.0
        self.Th = 0.0

        # Parámetros físicos
        self._l = 0.19   # Distancia entre ruedas
        self._r = 0.0505 # Radio de rueda

        # Factores de corrección
        self._angular_correction_factor = 0.90
        self._linear_correction_factor = 0.92

        # Otros parámetros
        self._sample_time = 0.01
        self.rate = 200.0

        # Estado interno
        self.first = True
        self.last_time = 0.0

        self.v_r = 0.0
        self.v_l = 0.0
        self.V = 0.0
        self.Omega = 0.0

        # Subscripciones
        self.sub_encR = self.create_subscription(Float32, 'VelocityEncR', self.encR_callback, qos.qos_profile_sensor_data)
        self.sub_encL = self.create_subscription(Float32, 'VelocityEncL', self.encL_callback, qos.qos_profile_sensor_data)

        # Publicador
        self.odom_pub = self.create_publisher(Odometry, 'odom', qos.qos_profile_sensor_data)

        # Timer
        self.timer = self.create_timer(1.0 / self.rate, self.run)

        # Mensajes
        self.odom_msg = Odometry()
        self.get_logger().info("🧭 Nodo OdometryNode iniciado.")

    def encR_callback(self, msg):
        self.v_r = self._r * msg.data

    def encL_callback(self, msg):
        self.v_l = self._r * msg.data

    def run(self):
        if self.first:
            self.last_time = self.get_clock().now()
            self.first = False
            return

        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9

        if dt >= self._sample_time:
            # Cinemática diferencial
            self.V = (self.v_r + self.v_l) / 2.0
            self.Omega = (self.v_r - self.v_l) / self._l

            # Actualización de pose con factores de corrección
            self.Th += self.Omega * dt * self._angular_correction_factor
            self.X += self.V * np.cos(self.Th) * dt * self._linear_correction_factor
            self.Y += self.V * np.sin(self.Th) * dt * self._linear_correction_factor

            self.last_time = now
            self.publish_odometry()

    def publish_odometry(self):
        q = transforms3d.euler.euler2quat(0, 0, self.Th)

        self.odom_msg.header.stamp = self.get_clock().now().to_msg()
        self.odom_msg.header.frame_id = 'odom'
        self.odom_msg.child_frame_id = 'base_footprint'

        self.odom_msg.pose.pose.position.x = self.X
        self.odom_msg.pose.pose.position.y = self.Y
        self.odom_msg.pose.pose.position.z = 0.0

        self.odom_msg.pose.pose.orientation.x = q[1]
        self.odom_msg.pose.pose.orientation.y = q[2]
        self.odom_msg.pose.pose.orientation.z = q[3]
        self.odom_msg.pose.pose.orientation.w = q[0]

        self.odom_msg.twist.twist.linear.x = self.V
        self.odom_msg.twist.twist.angular.z = self.Omega

        self.odom_pub.publish(self.odom_msg)

    def stop_handler(self, signum, frame):
        self.get_logger().info("❌ Interrupción recibida. Deteniendo nodo...")
        raise SystemExit


def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    signal.signal(signal.SIGINT, node.stop_handler)
    try:
        rclpy.spin(node)
    except SystemExit:
        node.get_logger().info('🛑 Apagado del nodo OdometryNode.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
