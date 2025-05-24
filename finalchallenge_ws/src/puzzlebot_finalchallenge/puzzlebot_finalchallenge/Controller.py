#!/usr/bin/env python3
# ------------------------------------------------------------------------------
# Proyecto: Puzzlebot Final Challenge - Controller – línea + semáforo, velocidad continua y anticipada
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
from geometry_msgs.msg import Twist
from std_msgs.msg     import Float32, String
import numpy as np


class Controller(Node):
    def __init__(self):
        super().__init__('Controller')

        # ─────── Parámetros ROS (puedes sobreescribir en YAML) ───────
        self.declare_parameter('kp_base', 0.0035)
        self.declare_parameter('kd',      0.0015)
        self.declare_parameter('v_max',   0.20)   # m/s en recta
        self.declare_parameter('v_min',   0.05)   # m/s en curva cerrada
        self.declare_parameter('ramp_step', 0.01) # m/s por tick (20 Hz)
        self.declare_parameter('alpha',     0.45)  # filtro derror
        self.declare_parameter('max_error', 40.0) # px (para clip)

        # ─────── Variables internas ───────
        self.error, self.prev_error = 0.0, 0.0
        self.derror = 0.0
        self.prev_time = self.get_clock().now()
        self.valid_error = False

        self.traffic_light_state = "none"
        self.current_speed = 0.0   # empieza parado

        # Suscripciones / publicación
        self.create_subscription(Float32, '/line_follower_data', self.cb_error, 10)
        self.create_subscription(String,  '/color_detector',     self.cb_color, 10)
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)

        self.create_timer(0.05, self.cb_timer)  # 20 Hz
        self.get_logger().info('🚗 Controller continuo ON')

    # ---------- Callbacks ----------
    def cb_error(self, msg: Float32):
        self.valid_error = not np.isnan(msg.data)
        if self.valid_error:
            self.error = msg.data

    def cb_color(self, msg: String):
        if msg.data in ("stop", "slow", "continue"):
            self.traffic_light_state = msg.data
        self.get_logger().info(f'🎨 Semáforo: {self.traffic_light_state}')

    # ---------- Bucle de control ----------
    def cb_timer(self):
        now = self.get_clock().now()
        dt = (now - self.prev_time).nanoseconds * 1e-9
        self.prev_time = now
        if dt <= 0:
            return

        # Sin línea → parada
        if not self.valid_error:
            self.publish_twist(0.0, 0.0)
            return

        # ----- PID para ángulo -----
        max_err = self.get_parameter('max_error').value
        err = np.clip(self.error, -max_err, max_err)

        kp = self.get_parameter('kp_base').value * (1 + abs(err)/max_err)
        kd = self.get_parameter('kd').value

        deriv = (err - self.prev_error) / dt
        self.prev_error = err

        # filtro
        alpha = self.get_parameter('alpha').value
        self.derror = alpha * deriv + (1 - alpha) * self.derror

        ang = -(kp * err + kd * deriv)
        ang = float(np.clip(ang, -1.2, 1.2))

        # ----- Velocidad lineal continua -----
        v_max = self.get_parameter('v_max').value
        v_min = self.get_parameter('v_min').value
        k_err   = (v_max - v_min) / max_err      # influencia del error
        k_derr  = 0.001                          # influencia de la derivada

        v_target = v_max - k_err*abs(err) - k_derr*abs(self.derror)
        v_target = np.clip(v_target, v_min, v_max)

        # Semáforo
        if self.traffic_light_state == "stop":
            v_target = 0.0
        elif self.traffic_light_state == "slow":
            v_target = min(v_target, 0.10)

        # Rampa
        step = self.get_parameter('ramp_step').value
        if self.current_speed < v_target:
            self.current_speed = min(self.current_speed + step, v_target)
        else:
            self.current_speed = max(self.current_speed - step, v_target)

        self.publish_twist(self.current_speed, ang)

    # ---------- Util ----------
    def publish_twist(self, v, w):
        msg = Twist()
        msg.linear.x  = v
        msg.angular.z = w
        self.pub_cmd.publish(msg)
        self.get_logger().info(
            f'Err {self.error:+5.1f}  dErr {self.derror:+6.1f}  '
            f'v {v:0.2f}  ang {w:0.2f}  ({self.traffic_light_state})'
        )


# ---------------- main ----------------
def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
