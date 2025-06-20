#!/usr/bin/env python3
# ------------------------------------------------------------------------------
# Proyecto: Puzzlebot Final Challenge - Nodo Controlador PD
# Materia: Implementación de Robótica Inteligente
# Fecha: 14 de junio de 2025
# Alumnos:
#   - Jonathan Arles Guevara Molina  | A01710380
#   - Ezzat Alzahouri Campos         | A01710709
#   - José Ángel Huerta Ríos         | A01710607
#   - Ricardo Sierra Roa             | A01709887
# ------------------------------------------------------------------------------
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, String, Bool
import numpy as np
import signal, sys

class Controller(Node):
    def __init__(self):
        super().__init__('Controller')

        # Declaración de parámetros del controlador PD
        self.declare_parameter('kp_base',   0.0035)
        self.declare_parameter('kd',        0.0015)
        self.declare_parameter('v_max',     0.15)   # m/s recta
        self.declare_parameter('v_min',     0.04)   # m/s curva cerrada & slow-mode
        self.declare_parameter('ramp_step', 0.01)   # m/s por tick (20 Hz)
        self.declare_parameter('alpha',     0.45)   # filtro derror
        self.declare_parameter('max_error', 40.0)   # px

        # Variables internas del controlador
        self.error, self.prev_error = 0.0, 0.0
        self.derror = 0.0
        self.prev_time = self.get_clock().now()
        self.valid_error = False

        # Estado del entorno y decisiones del robot
        self.traffic_light_state = "none"
        self.current_speed = 0.0
        self.ready_to_go = False
        self.zebra_detected = False
        self.signal_detected = "none"
        self.path_following = False

        # Tiempos para modos especiales
        self.slow_end_time = None
        self.stop_end_time = None


        # Suscriptores a los distintos tópicos de nodos
        self.create_subscription(Float32, '/line_follower_data', self.cb_error,   10)
        self.create_subscription(String,  '/color_detector',     self.cb_color,   10)
        self.create_subscription(String,  '/signal_detector',    self.cb_signal,  10)
        self.create_subscription(Bool,    '/zebra_detected',     self.cb_zebra,   10)
        self.create_subscription(String,  '/path_done',          self.cb_path_done, 10)

        # Publicadores de velocidad y dirección
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_dir = self.create_publisher(String, '/go_direction', 10)

        # Manejo de interrupciones para apagado
        signal.signal(signal.SIGINT, self.shutdown_function)

        # Timer principal que ejecuta el bucle de control (20 Hz)
        self.create_timer(0.05, self.cb_timer)
        self.get_logger().info('🚗 Controller continuo ON')

    # Callback que recibe el error lateral del seguidor de línea
    def cb_error(self, msg: Float32):
        self.valid_error = not np.isnan(msg.data)
        if self.valid_error:
            self.error = msg.data

    # Callback cuando termina un cambio de trayectoria
    def cb_path_done(self, msg: String):
        if msg.data == "done":
            self.zebra_detected = False
            self.path_following = False
            self.current_speed = 0.0

    # Callback cuando se detecta una intersección
    def cb_zebra(self, msg: Bool):
        self.zebra_detected = msg.data
        self.get_logger().info(f'Cebra detectada: {self.zebra_detected}')

    # Callback de detección de señales de tráfico (stop, give_way, worker_ahead, turn_left, turn_right, straight)
    def cb_signal(self, msg: String):
        self.signal_detected = msg.data
        now = self.get_clock().now()
        if msg.data == "worker_ahead":
            self.slow_end_time = now + rclpy.duration.Duration(seconds=10)
            self.get_logger().info("worker_ahead -> slow-mode 10 s")
        elif msg.data == "give_way":
            self.slow_end_time = now + rclpy.duration.Duration(seconds=5)
            self.get_logger().info("give_way -> slow-mode 5 s")
        elif msg.data == "stop":
            self.stop_end_time = now + rclpy.duration.Duration(seconds=10)
            self.get_logger().info("STOP -> detención 10 s")

    # Callback del detector de semáforo (rojo, amarillo, verde)
    def cb_color(self, msg: String):
        if msg.data in ("red", "yellow", "green"):
            self.traffic_light_state = msg.data
            if msg.data == "green" and not self.ready_to_go:
                self.ready_to_go = True
                self.get_logger().info("Semáforo verde → listos para avanzar")
        self.get_logger().info(f'Semáforo: {self.traffic_light_state}')

    # Bucle principal de control: evalúa errores, estados y actúa en consecuencia
    def cb_timer(self):
        now = self.get_clock().now()
        dt = (now - self.prev_time).nanoseconds * 1e-9
        self.prev_time = now
        if dt <= 0: return
        if self.path_following: return

        # Stop si condiciones no son seguras
        if (not self.valid_error or not self.ready_to_go or self.traffic_light_state == "red"):
            self.publish_twist(0.0, 0.0)
            return
        
        # Stop por señal de tráfico
        if self.stop_end_time:
            if now < self.stop_end_time:
                self.publish_twist(0.0, 0.0)
                return
            else:
                self.stop_end_time = None  # se acabó el stop

        # Cambio de trayectoria al detectar cebra + señal + verde
        if (self.zebra_detected and
            self.signal_detected in ("straight", "turn_left", "turn_right") and
            not self.path_following and
            self.traffic_light_state == "green"):
            self.path_following = True
            self.current_speed = 0.0
            self.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.5))
            self.pub_dir.publish(String(data=self.signal_detected))
            return

        # Cálculo del PD adaptativo
        max_err = self.get_parameter('max_error').value
        err = float(np.clip(self.error, -max_err, max_err))

        kp = self.get_parameter('kp_base').value * (1 + abs(err) / max_err)
        kd = self.get_parameter('kd').value

        deriv = (err - self.prev_error) / dt
        self.prev_error = err

        alpha = self.get_parameter('alpha').value
        self.derror = alpha * deriv + (1 - alpha) * self.derror

        ang = - (kp * err + kd * deriv)
        ang = float(np.clip(ang, -1.2, 1.2))

        # Cálculo de velocidad lineal según error
        v_max = self.get_parameter('v_max').value
        v_min = self.get_parameter('v_min').value
        k_err  = (v_max - v_min) / max_err
        k_derr = 0.001

        v_target = v_max - k_err * abs(err) - k_derr * abs(self.derror)
        v_target = float(np.clip(v_target, v_min, v_max))

        # Modo precaución si semáforo amarillo
        if self.traffic_light_state == "yellow":
            v_target = min(v_target, 0.03)

        # Reducción de velocidad si está activo el slow-mode
        if self.slow_end_time and now < self.slow_end_time:
            v_target = v_min  # disminuye velocidad
        elif self.slow_end_time and now >= self.slow_end_time:
            self.slow_end_time = None  # se acabó el tiempo de slow-mode

        # Aplicar rampa suave para evitar saltos bruscos en la velocidad
        step = self.get_parameter('ramp_step').value
        if self.current_speed < v_target:
            self.current_speed = min(self.current_speed + step, v_target)
        else:
            self.current_speed = max(self.current_speed - step, v_target)

        self.publish_twist(self.current_speed, ang)

    # Publica el comando de velocidad y dirección en el topic '/cmd_vel'-
    def publish_twist(self, v: float, w: float):
        msg = Twist()
        msg.linear.x  = v
        msg.angular.z = w
        self.pub_cmd.publish(msg)
        self.get_logger().info(f'Err {self.error:+5.1f} dErr {self.derror:+6.1f}' f'v {v:0.2f} ang {w:0.2f} ({self.traffic_light_state})')
        return msg
    
    # Función para detener el robot y apagar ROS2 correctamente
    def shutdown_function(self, *_):
        self.get_logger().info("Shutting down. Stopping robot…")
        self.publish_twist(0.0, 0.0)
        rclpy.shutdown()
        sys.exit(0)

# ---------------- main ----------------
def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
