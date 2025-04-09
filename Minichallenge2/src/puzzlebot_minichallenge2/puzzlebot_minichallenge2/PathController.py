import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from custom_interfaces.msg import PathPose
import numpy as np
import time

class PathController(Node):
    def __init__(self):
        super().__init__('path_controller')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pose_sub = self.create_subscription(PathPose, '/pose', self.pose_callback, 10)

        # Lista de puntos
        self.points = []

        # Modo TIME o SPEED
        self.mode = None
        self.total_time = 0.0
        self.speed_param = 0.0

        # FSM
        self.state = 'WAITING'
        self.current_segment = 0
        self.start_time = None

        # Config base
        self.base_lin_speed = 0.2   # m/s base
        self.base_ang_speed = 0.3   # rad/s base
        self.base_pause = 0.0       # ELIMINAMOS PAUSA

        # Velocidades y pausas ajustadas
        self.lin_speed = self.base_lin_speed
        self.ang_speed = self.base_ang_speed
        self.pause_time = self.base_pause

        # Guardamos tiempos por tramo
        self.turn_times = []    # t_turn[i]
        self.forward_times = [] # t_forward[i]
        self.num_segments = 0

        # Menos tiempo entre iteraciones => menos error
        self.timer_period = 0.05  
        self.timer = self.create_timer(self.timer_period, self.control_loop)
        self.get_logger().info("Combina giros con atan2 + tiempo fijo, sin pausas, timer=0.05s.")

    def pose_callback(self, msg):
        # Determinamos modo en el primer punto
        if self.mode is None:
            if msg.time_to_reach > 0:
                self.mode = 'time'
                self.total_time = msg.time_to_reach
                self.get_logger().info(f"⏱ Modo TIME => {self.total_time:.2f}s total")
            else:
                self.mode = 'speed'
                self.speed_param = msg.linear_velocity
                self.get_logger().info(f"🏎 Modo SPEED => {self.speed_param:.2f} m/s")

        x = msg.pose.position.x
        y = msg.pose.position.y
        self.points.append((x, y))
        self.get_logger().info(f"➕ Recibido punto {len(self.points)}: ({x:.2f}, {y:.2f})")

    def control_loop(self):
        now = time.time()

        if self.state == 'WAITING':
            # Requerimos al menos 2 puntos
            if len(self.points) >= 2 and self.mode is not None:
                self.setup_trajectory()
                self.current_segment = 0
                self.state = 'TURN'
                self.start_time = now

        elif self.state == 'TURN':
            if self.current_segment >= self.num_segments:
                self.stop_robot()
                self.state = 'DONE'
                self.get_logger().info("🏁 Ruta completada (TURN).")
                return

            turn_t = self.turn_times[self.current_segment]
            elapsed = now - self.start_time

            if elapsed < turn_t:
                twist = Twist()
                sign = 1.0 if self.angles_signo[self.current_segment] >= 0 else -1.0
                twist.angular.z = self.ang_speed * sign
                self.cmd_pub.publish(twist)
            else:
                self.stop_robot()
                self.start_time = now
                self.state = 'FORWARD'
                self.get_logger().info(f"🌀 Giro {self.current_segment+1} completado.")

        elif self.state == 'FORWARD':
            if self.current_segment >= self.num_segments:
                self.stop_robot()
                self.state = 'DONE'
                self.get_logger().info("🏁 Ruta completada (FORWARD).")
                return

            forward_t = self.forward_times[self.current_segment]
            elapsed = now - self.start_time

            if elapsed < forward_t:
                twist = Twist()
                twist.linear.x = self.lin_speed
                self.cmd_pub.publish(twist)
            else:
                self.stop_robot()
                self.current_segment += 1
                self.get_logger().info(f"✅ Tramo {self.current_segment} completado.")
                if self.current_segment < self.num_segments:
                    self.state = 'TURN'
                    self.start_time = time.time()
                else:
                    self.state = 'DONE'
                    self.get_logger().info("🏁 Ruta completada (FORWARD).")

        elif self.state == 'DONE':
            pass

    def setup_trajectory(self):
        """Calcula giros y rectas con atan2 y ajusta speeds sin pausas."""
        self.turn_times.clear()
        self.forward_times.clear()
        self.angles_signo = []
        sum_dist = 0.0
        sum_angle = 0.0

        if len(self.points) < 2:
            return

        dx0 = self.points[1][0] - self.points[0][0]
        dy0 = self.points[1][1] - self.points[0][1]
        prev_dir = np.arctan2(dy0, dx0)

        self.distances = []
        for i in range(len(self.points)-1):
            p0 = self.points[i]
            p1 = self.points[i+1]
            dx = p1[0] - p0[0]
            dy = p1[1] - p0[1]
            dist_i = np.hypot(dx, dy)
            self.distances.append(dist_i)
            sum_dist += dist_i

            new_dir = np.arctan2(dy, dx)
            angle_diff = self.normalize_angle(new_dir - prev_dir)
            sign = 1.0 if angle_diff>=0 else -1.0
            angle_abs = abs(angle_diff)
            self.angles_signo.append(sign)
            sum_angle += angle_abs

            prev_dir = new_dir

        self.num_segments = len(self.distances)

        if self.mode == 'time':
            base_lin_time = sum_dist / self.base_lin_speed if self.base_lin_speed>0 else 0.0
            base_ang_time = sum_angle / self.base_ang_speed if self.base_ang_speed>0 else 0.0
            # Sin pausas
            T_base = base_lin_time + base_ang_time

            if T_base < 1e-6:
                self.lin_speed = 0.0
                self.ang_speed = 0.0
            else:
                scale = T_base / self.total_time
                self.lin_speed = self.base_lin_speed / scale
                self.ang_speed = self.base_ang_speed / scale

            self.get_logger().info(
                f"[TIME] => T_base={T_base:.2f}, scale={scale:.2f}, "
                f"lin={self.lin_speed:.2f}, ang={self.ang_speed:.2f}"
            )
        else:
            # speed => lin_speed = speed_param, ang_speed base
            self.lin_speed = self.speed_param
            self.ang_speed = self.base_ang_speed

            self.get_logger().info(
                f"[SPEED] => lin={self.lin_speed:.2f}, ang={self.ang_speed:.2f}"
            )

        # Asignar turn/forward times
        prev_dir = np.arctan2(dy0, dx0)
        for i in range(self.num_segments):
            angle_diff = abs(self.normalize_angle(np.arctan2(
                self.points[i+1][1]-self.points[i][1],
                self.points[i+1][0]-self.points[i][0]
            ) - prev_dir))
            turn_t = angle_diff / self.ang_speed if self.ang_speed>0 else 0.0
            self.turn_times.append(turn_t)

            dist_i = self.distances[i]
            forward_t = dist_i / self.lin_speed if self.lin_speed>0 else 0.0
            self.forward_times.append(forward_t)

            new_dir = np.arctan2(
                self.points[i+1][1]-self.points[i][1],
                self.points[i+1][0]-self.points[i][0]
            )
            prev_dir = new_dir

        t_calc = sum(self.turn_times)+sum(self.forward_times)
        self.get_logger().info(
            f"Suma de turn_times+forward_times={t_calc:.2f} (total_time={self.total_time:.2f})"
        )

    def stop_robot(self):
        self.cmd_pub.publish(Twist())

    def normalize_angle(self, angle):
        return np.arctan2(np.sin(angle), np.cos(angle))

def main(args=None):
    rclpy.init(args=args)
    node = PathController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
