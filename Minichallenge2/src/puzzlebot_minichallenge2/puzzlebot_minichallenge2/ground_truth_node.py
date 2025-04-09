#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy  # <-- Importar QoS
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion, Twist, Vector3
import numpy as np

class GroundTruthPublisher(Node):
    def __init__(self):
        super().__init__('ground_truth_publisher')

        # Parámetros físicos
        self.L = 0.19  # distancia entre ruedas
        self.K = 0.0505   # factor para escalar velocidad si es necesario

        # Estado
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.vl = 0.0
        self.vr = 0.0

        # Guardamos el tiempo inicial
        self.last_time = self.get_clock().now()

        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT

        # Subscripciones
        self.create_subscription(Float32, '/VelocityEncL', self.left_cb, qos_profile)
        self.create_subscription(Float32, '/VelocityEncR', self.right_cb, qos_profile)

        # Publicador en /ground_truth (aquí podemos dejarlo con QoS por defecto)
        self.pub = self.create_publisher(Odometry, '/ground_truth', 10)

        # Timer a 20 Hz
        self.timer = self.create_timer(0.05, self.timer_callback)

        self.get_logger().info("Nodo ground_truth iniciado.")

    def left_cb(self, msg):
        self.vl = msg.data * self.K
        #self.get_logger().info(f"[left_cb] Recibido vl= {msg.data:.3f}, escalada= {self.vl:.3f}")

    def right_cb(self, msg):
        self.vr = msg.data * self.K
        #self.get_logger().info(f"[right_cb] Recibido vr= {msg.data:.3f}, escalada= {self.vr:.3f}")

    def timer_callback(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        # Cinemática diferencial
        v = (self.vr + self.vl) / 2.0
        w = (self.vr - self.vl) / self.L

        dx = v * np.cos(self.theta) * dt
        dy = v * np.sin(self.theta) * dt
        dtheta = w * dt

        self.x += dx
        self.y += dy
        self.theta += dtheta

        # Construir mensaje Odometry
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = "ground_truth"
        odom.child_frame_id = "base_link"

        # Pose
        odom.pose.pose.position = Point(x=self.x, y=self.y, z=0.0)
        q = self.quaternion_from_euler(0.0, 0.0, self.theta)
        odom.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        # Twist
        odom.twist.twist = Twist(
            linear=Vector3(x=v, y=0.0, z=0.0),
            angular=Vector3(x=0.0, y=0.0, z=w)
        )

        # Publicar en /ground_truth
        self.pub.publish(odom)

    def quaternion_from_euler(self, roll, pitch, yaw):
        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)
        cp = np.cos(pitch * 0.5)
        sp = np.sin(pitch * 0.5)
        cr = np.cos(roll * 0.5)
        sr = np.sin(roll * 0.5)

        return [
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy
        ]

def main(args=None):
    rclpy.init(args=args)
    node = GroundTruthPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
