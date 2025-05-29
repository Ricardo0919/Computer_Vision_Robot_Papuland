#!/usr/bin/env python3
# ------------------------------------------------------------------------------
# Proyecto: Navegación de Línea con Control Difuso Basado en Centroide - Fuzzy Controller
# Materia: Implementación de Robótica Inteligente
# Fecha: 6 de junio de 2025
# Alumnos:
#   - Jonathan Arles Guevara Molina  | A01710380
#   - Ezzat Alzahouri Campos         | A01710709
#   - José Ángel Huerta Ríos         | A01710607
#   - Ricardo Sierra Roa             | A01709887
# ------------------------------------------------------------------------------

import rclpy
import signal
import sys
import numpy as np
import skfuzzy as fuzz
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, String
from skfuzzy import control as ctrl


class Controller(Node):
    def __init__(self):
        super().__init__('Controller_fuzzy')

        # ── Velocidades
        self.declare_parameter('v_max', 0.05)   # recta
        self.declare_parameter('v_min', 0.02)   # curva lenta
        self.declare_parameter('ramp_step', 0.01)
        self.declare_parameter('max_error', 40.0)
        self.declare_parameter('alpha', 0.45)

        # ── Estado
        self.error = self.prev_error = 0.0
        self.d_error = 0.0
        self.prev_time = self.get_clock().now()
        self.current_speed = 0.0
        self.valid_error = False
        self.ready_to_go = False
        self.traffic_light_state = "none"
        self.ang_filt = 0.0

        # ROS I/O
        self.create_subscription(Float32, '/line_follower_data',
                                 self.cb_error, 10)
        self.create_subscription(String, '/color_detector',
                                 self.cb_color, 10)
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)

        self._build_fuzzy()
        signal.signal(signal.SIGINT, self.shutdown)
        self.create_timer(0.05, self.cb_timer)    # 20 Hz
        self.get_logger().info('🚗 Fuzzy-PD amortiguado listo')

    # ──────────────────────────
    def _build_fuzzy(self):
        E  = np.linspace(-60, 60, 121)
        dE = np.linspace(-300, 300, 121)
        W  = np.linspace(-0.3, 0.3, 101)

        err, derr = ctrl.Antecedent(E,'err'), ctrl.Antecedent(dE,'derr')
        omega     = ctrl.Consequent(W,'omega', defuzzify_method='centroid')

        # Error (dead-band ±15 px)
        err['NL']=fuzz.trapmf(E,[-60,-60,-40,-20])
        err['NS']=fuzz.trimf(E,[-40,-20,   0])
        err['Z'] =fuzz.trapmf(E,[-15, -3,   3, 15])   # ⇦
        err['PS']=fuzz.trimf(E,[  0, 20,  40])
        err['PL']=fuzz.trapmf(E,[ 20, 40,  60, 60])

        # dError (gauss estrecha en el centro)
        derr['NL']=fuzz.trapmf(dE,[-300,-300,-180,-60])
        derr['NS']=fuzz.trimf(dE, [-180,-60,   0])
        derr['Z'] =fuzz.gaussmf(dE, 0, 25)            # ⇦
        derr['PS']=fuzz.trimf(dE, [  0, 60, 180])
        derr['PL']=fuzz.trapmf(dE,[ 60,180,300,300])

        # omega (sin cambios)
        omega['NL'] = fuzz.trapmf(W, [-0.20, -0.20, -0.14, -0.07])
        omega['NS'] = fuzz.trimf(W,  [-0.14, -0.07,  0.00])
        omega['Z']  = fuzz.trimf(W,  [-0.02,  0.00,  0.02])
        omega['PS'] = fuzz.trimf(W,  [ 0.00,  0.07,  0.14])
        omega['PL'] = fuzz.trapmf(W, [ 0.07,  0.14,  0.20, 0.20])

        rules = [
            ctrl.Rule(err['NL'], omega['PL']),
            ctrl.Rule(err['NS'], omega['PS']),
            ctrl.Rule(err['Z'] , omega['Z']),
            ctrl.Rule(err['PS'], omega['NS']),
            ctrl.Rule(err['PL'], omega['NL']),
            ctrl.Rule(err['NS'] & derr['NL'], omega['Z']),
            ctrl.Rule(err['PS'] & derr['PL'], omega['Z']),
            ctrl.Rule(err['Z']  & derr['Z'] , omega['Z']),   # ⇦ nuevo
            ctrl.Rule(err['Z']  & derr['NS'], omega['Z'])    # ⇦ nuevo
        ]
        self.fuzzy = ctrl.ControlSystemSimulation(ctrl.ControlSystem(rules))




    # ───────── Callbacks ─────────
    def cb_error(self, msg: Float32):
        self.valid_error = not np.isnan(msg.data)
        if self.valid_error:
            self.error = msg.data

    def cb_color(self, msg: String):
        if msg.data in ("red", "yellow", "green"):
            self.traffic_light_state = msg.data
            if msg.data == "green":
                self.ready_to_go = True

    # ───────── Bucle de control ─────────
    def cb_timer(self):
        now = self.get_clock().now()
        dt  = (now - self.prev_time).nanoseconds * 1e-9
        self.prev_time = now

        if dt <= 0 or not self.valid_error or not self.ready_to_go \
           or self.traffic_light_state == "red":
            self.publish_twist(0.0, 0.0)
            return

        deriv = (self.error - self.prev_error) / dt
        self.prev_error = self.error
        alpha = self.get_parameter('alpha').value
        self.d_error = alpha*deriv + (1-alpha)*self.d_error

        self.fuzzy.input['err']  = np.clip(self.error,  -60,  60)
        self.fuzzy.input['derr'] = np.clip(self.d_error,-300, 300)
        self.fuzzy.compute()
        ang_raw = float(self.fuzzy.output['omega'])

        # corte duro
        if abs(self.error) < 4 and abs(self.d_error) < 15:
            ang_raw = 0.0

        # filtro más suave
        self.ang_filt = 0.95*self.ang_filt + 0.05*ang_raw

        # Velocidad
        v_max   = self.get_parameter('v_max').value
        v_min   = self.get_parameter('v_min').value
        max_err = self.get_parameter('max_error').value

        ratio    = abs(self.error) / max_err
        k_derr   = 0.001                               # (5)
        v_target = v_max - (v_max - v_min)*(ratio**0.5) \
                   - k_derr*abs(self.d_error)
        v_target = np.clip(v_target, v_min, v_max)

        if self.traffic_light_state == "yellow":
            v_target = min(v_target, 0.05)

        step = self.get_parameter('ramp_step').value
        self.current_speed += np.sign(v_target - self.current_speed) \
                              * min(abs(v_target - self.current_speed), step)

        self.publish_twist(self.current_speed, self.ang_filt)

    # ───────── Util ─────────
    def publish_twist(self, v, w):
        msg = Twist()
        msg.linear.x  = v
        msg.angular.z = w
        self.pub_cmd.publish(msg)
        self.get_logger().info(
            f'Err {self.error:+6.1f}  dErr {self.d_error:+7.1f}  '
            f'v {v:0.3f}  ω {w:0.3f}')

    def shutdown(self, *_):
        self.pub_cmd.publish(Twist())
        rclpy.shutdown()
        sys.exit(0)


def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
