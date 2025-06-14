#!/usr/bin/env python3
# ------------------------------------------------------------------------------
# Proyecto: Puzzlebot Final Challenge - Nodo de Odometría (con reset)
# Materia: Implementación de Robótica Inteligente
# Fecha: 14 de junio de 2025
# Alumnos:
#   - Jonathan Arles Guevara Molina  | A01710380
#   - Ezzat Alzahouri Campos         | A01710709
#   - José Ángel Huerta Ríos         | A01710607
#   - Ricardo Sierra Roa             | A01709887
# ------------------------------------------------------------------------------

import rclpy, numpy as np, signal
from rclpy.node import Node
from rclpy import qos
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty  

class OdometryNode(Node):
    def __init__(self):
        super().__init__('OdometryNode')

        # Variables de estado del robot
        self.X = self.Y = self.Th = 0.0                  # Posición y orientación
        self.v_r = self.v_l = self.V = self.Omega = 0.0  # Velocidades ruedas y robot

        # Parámetros físicos del robot
        self._l = 0.19          # Distancia entre ruedas (m)
        self._r = 0.0505        # Radio de las ruedas (m)

        # Factores de corrección configurables
        self.declare_parameter('angular_correction_factor', 0.91)
        self.declare_parameter('linear_correction_factor', 0.90)
        self.cf_ang = self.get_parameter('angular_correction_factor').value
        self.cf_lin = self.get_parameter('linear_correction_factor').value

        # Temporización del loop de odometría
        self._sample_dt = 0.01
        self.rate_hz    = 200.0
        self.first      = True
        self.last_time  = 0.0

        # Subscripción a velocidades de encoders
        self.create_subscription(Float32, 'VelocityEncR', self.cb_encR, qos.qos_profile_sensor_data)
        self.create_subscription(Float32, 'VelocityEncL', self.cb_encL, qos.qos_profile_sensor_data)
        
        # Publicador de odometría
        self.odom_pub = self.create_publisher(Odometry, 'odom', qos.qos_profile_sensor_data)
        self.odom_msg = Odometry()

        # Timer para actualizar estado del robot
        self.create_timer(1.0 / self.rate_hz, self.update)

        # Servicio para resetear la odometría
        self.create_service(Empty, 'reset_odometry', self.reset_cb)

        self.get_logger().info("🧭 OdometryNode iniciado")

    # Callback para leer velocidad del encoder derecho
    def cb_encR(self, msg):  
        self.v_r =  self._r * msg.data
    
    # Callback para leer velocidad del encoder izquierdo
    def cb_encL(self, msg):  
        self.v_l =  self._r * msg.data

    # Callback del servicio para reiniciar la odometría
    def reset_cb(self, req, res):
        self.X = self.Y = self.Th = 0.0
        self.v_r = self.v_l = self.V = self.Omega = 0.0
        self.first = True
        self.get_logger().info("🔄 Odometría reiniciada")
        return res

    # Loop de odometría: integra velocidades para actualizar pose
    def update(self):
        if self.first:
            self.last_time = self.get_clock().now()
            self.first = False
            return

        now = self.get_clock().now()
        dt  = (now - self.last_time).nanoseconds * 1e-9
        if dt < self._sample_dt:
            return

        # Cálculo de velocidad lineal y angular
        self.V     = (self.v_r + self.v_l) / 2.0
        self.Omega = (self.v_r - self.v_l) / self._l

        # Integración para obtener nueva posición
        self.Th += self.Omega * dt * self.cf_ang
        self.X  += self.V * np.cos(self.Th) * dt * self.cf_lin
        self.Y  += self.V * np.sin(self.Th) * dt * self.cf_lin

        self.last_time = now
        self.publish()

    # Conversión de ángulos de Euler a cuaterniones
    @staticmethod
    def euler_to_quat(r, p, y):
        cy, sy = np.cos(y*0.5), np.sin(y*0.5)
        cp, sp = np.cos(p*0.5), np.sin(p*0.5)
        cr, sr = np.cos(r*0.5), np.sin(r*0.5)
        qw = cr*cp*cy + sr*sp*sy
        qx = sr*cp*cy - cr*sp*sy
        qy = cr*sp*cy + sr*cp*sy
        qz = cr*cp*sy - sr*sp*cy
        return qx,qy,qz,qw  # x,y,z,w

    # Publicar mensaje de odometría
    def publish(self):
        qx,qy,qz,qw = self.euler_to_quat(0,0,self.Th)
        m = self.odom_msg
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'odom'
        m.child_frame_id  = 'base_footprint'

        m.pose.pose.position.x = self.X
        m.pose.pose.position.y = self.Y
        m.pose.pose.orientation.x = qx
        m.pose.pose.orientation.y = qy
        m.pose.pose.orientation.z = qz
        m.pose.pose.orientation.w = qw
        m.twist.twist.linear.x  = self.V
        m.twist.twist.angular.z = self.Omega

        self.odom_pub.publish(m)

    # Manejador de ctrl+c para apagado
    def stop_handler(self,*_):
        raise SystemExit

# ---------------- main ----------------
def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    signal.signal(signal.SIGINT, node.stop_handler)
    try: rclpy.spin(node)
    except SystemExit: pass
    node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()
