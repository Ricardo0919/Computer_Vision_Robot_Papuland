#!/usr/bin/env python3
# ------------------------------------------------------------
# Puzzlebot Final Challenge – Nodo de Odometría (con reset)
# ------------------------------------------------------------
import rclpy, numpy as np, signal
from rclpy.node import Node
from rclpy import qos
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty  

class OdometryNode(Node):
    def __init__(self):
        super().__init__('OdometryNode')

        # --- pose & velocidades ---
        self.X = self.Y = self.Th = 0.0
        self.v_r = self.v_l = self.V = self.Omega = 0.0

        # --- parámetros físicos ---
        self._l = 0.19
        self._r = 0.0505

        # --- factores de corrección (param ROS) ---
        self.declare_parameter('angular_correction_factor', 0.91)
        self.declare_parameter('linear_correction_factor', 0.90)
        self.cf_ang = self.get_parameter('angular_correction_factor').value
        self.cf_lin = self.get_parameter('linear_correction_factor').value

        # --- temporización ---
        self._sample_dt = 0.01
        self.rate_hz    = 200.0
        self.first      = True
        self.last_time  = 0.0

        # --- topics ---
        self.create_subscription(Float32, 'VelocityEncR',
                                 self.cb_encR, qos.qos_profile_sensor_data)
        self.create_subscription(Float32, 'VelocityEncL',
                                 self.cb_encL, qos.qos_profile_sensor_data)
        self.odom_pub = self.create_publisher(Odometry, 'odom',
                                              qos.qos_profile_sensor_data)
        self.odom_msg = Odometry()

        # --- timer principal ---
        self.create_timer(1.0 / self.rate_hz, self.update)

        # --- servicio reset ---
        self.create_service(Empty, 'reset_odometry', self.reset_cb)

        self.get_logger().info("🧭 OdometryNode iniciado")

    # ---------- callbacks ----------
    def cb_encR(self, msg):  self.v_r =  self._r * msg.data
    def cb_encL(self, msg):  self.v_l =  self._r * msg.data  # invierte signo si fuese necesario

    def reset_cb(self, req, res):
        self.X = self.Y = self.Th = 0.0
        self.v_r = self.v_l = self.V = self.Omega = 0.0
        self.first = True                          # reiniciar temporizador
        self.get_logger().info("🔄 Odometría reiniciada")
        return res

    # ---------- update ----------
    def update(self):
        if self.first:
            self.last_time = self.get_clock().now()
            self.first = False
            return

        now = self.get_clock().now()
        dt  = (now - self.last_time).nanoseconds * 1e-9
        if dt < self._sample_dt:
            return

        self.V     = (self.v_r + self.v_l) / 2.0
        self.Omega = (self.v_r - self.v_l) / self._l

        self.Th += self.Omega * dt * self.cf_ang
        self.X  += self.V * np.cos(self.Th) * dt * self.cf_lin
        self.Y  += self.V * np.sin(self.Th) * dt * self.cf_lin

        self.last_time = now
        self.publish()

    # ---------- publicar odom ----------
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

    # ---------- ctrl-c ----------
    def stop_handler(self,*_):
        raise SystemExit

def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    signal.signal(signal.SIGINT, node.stop_handler)
    try: rclpy.spin(node)
    except SystemExit: pass
    node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()
