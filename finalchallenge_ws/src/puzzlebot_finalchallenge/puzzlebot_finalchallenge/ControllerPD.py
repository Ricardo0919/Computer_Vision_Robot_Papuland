#!/usr/bin/env python3
# ------------------------------------------------------------------------------
# Proyecto: Puzzlebot Final Challenge - Nodo Controlador (PD) en base al seguidor de linea y color de semaforo 
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
from std_msgs.msg     import Float32, String, Bool
import numpy as np
import signal  
import sys 


class Controller(Node):
    def __init__(self):
        super().__init__('Controller')

        # ─────── Parámetros ROS (puedes sobreescribir en YAML) ───────
        self.declare_parameter('kp_base', 0.0035)
        self.declare_parameter('kd',      0.0015)
        self.declare_parameter('v_max',   0.15)    # m/s en recta
        self.declare_parameter('v_min',   0.04)   # m/s en curva cerrada
        self.declare_parameter('ramp_step', 0.01) # m/s por tick (20 Hz)
        self.declare_parameter('alpha',     0.45) # filtro derror
        self.declare_parameter('max_error', 40.0) # px (para clip)

        # ─────── Variables internas ───────
        self.error, self.prev_error = 0.0, 0.0
        self.derror = 0.0
        self.prev_time = self.get_clock().now()
        self.valid_error = False

        self.traffic_light_state = "none"
        self.current_speed = 0.0     # empieza parado
        self.ready_to_go = False     # Nueva bandera: solo se activa al recibir "green"
        self.zebra_detected = False  # Variable para detectar cebra



        # Suscripciones / publicación
        self.create_subscription(Float32, '/line_follower_data', self.cb_error, 10)
        self.create_subscription(String,  '/color_detector',     self.cb_color, 10)
        self.create_subscription(Bool,  '/zebra_detected',     self.cb_zebra, 10)
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)

        # Handle shutdown gracefully 

        signal.signal(signal.SIGINT, self.shutdown_function) # When Ctrl+C is pressed, call self.shutdown_function 

        self.create_timer(0.05, self.cb_timer)  # 20 Hz
        self.get_logger().info('🚗 Controller continuo ON')

    # ---------- Callbacks ----------
    def cb_error(self, msg: Float32):
        self.valid_error = not np.isnan(msg.data)
        if self.valid_error:
            self.error = msg.data

    def cb_zebra(self, msg: Bool):
        self.zebra_detected = msg.data
        self.get_logger().info(f'Cebra detectada: {self.zebra_detected}')


    def cb_color(self, msg: String):
        if msg.data in ("red", "yellow", "green"):
            self.traffic_light_state = msg.data
            # Activar solo una vez cuando llega el verde por primera vez
            if msg.data == "green" and not self.ready_to_go:
                self.ready_to_go = True
                self.get_logger().info("Semáforo verde recibido, Robot listo para avanzar.")
        self.get_logger().info(f'Semáforo: {self.traffic_light_state}')


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
        
        # Si el semáforo no está verde al iniciar, no avanzar
        if not self.ready_to_go:
            self.publish_twist(0.0, 0.0)
            return
        
        # Si el semáforo es rojo, detener el robot
        if self.traffic_light_state == "red":
            self.publish_twist(0.0, 0.0)
            return

        # Si hay cebra detectada, detener el robot
        if self.zebra_detected:
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
        if self.traffic_light_state == "yellow":
            v_target = min(v_target, 0.05)

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

    def shutdown_function(self, signum, frame): 
        # Handle shutdown gracefully 
        # This function will be called when Ctrl+C is pressed 
        # It will stop the robot and shutdown the node 
        self.get_logger().info("Shutting down. Stopping robot...") 
        stop_twist = self.publish_twist(0.0, 0.0)  # All zeros to stop the robot 
        self.pub_cmd.publish(stop_twist) # publish it to stop the robot before shutting down 
        rclpy.shutdown() # Shutdown the node 
        sys.exit(0) # Exit the program 

# ---------------- main ----------------
def main(args=None): 
    rclpy.init(args=args) 
    my_node=Controller() 
    rclpy.spin(my_node) 
    my_node.destroy_node() 
    rclpy.shutdown() 

if __name__ == '__main__': 
    main() 