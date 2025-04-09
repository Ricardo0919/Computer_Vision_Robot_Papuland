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

        self.goals = []
        self.state = 'IDLE'
        self.current_index = 0
        self.start_time = None
        self.phase_duration = 0.0

        # Simulación de orientación
        self.yaw = 0.0  # en radianes
        self.angular_speed = 0.6  # rad/s
        self.linear_speed_default = 0.15  # m/s por defecto si es 'time'

        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("✅ PathController iniciado.")

    def pose_callback(self, msg):
        self.goals.append({
            'x': msg.pose.position.x,
            'y': msg.pose.position.y,
            'time': msg.time_to_reach,
            'speed': msg.linear_velocity
        })

        self.get_logger().info(f"📍 Punto recibido: ({msg.pose.position.x}, {msg.pose.position.y}) "
                               f"[modo={'time' if msg.time_to_reach > 0 else 'speed'}]")

        if self.state == 'IDLE' and len(self.goals) > 0:
            self.state = 'TURN'
            self.start_time = time.time()

    def control_loop(self):
        if self.state == 'IDLE' or self.current_index >= len(self.goals):
            return

        now = time.time()
        elapsed = now - self.start_time
        goal = self.goals[self.current_index]
        vel = Twist()

        if self.state == 'TURN':
            if self.current_index == 0:
                dx = goal['x']
                dy = goal['y']
            else:
                prev = self.goals[self.current_index - 1]
                dx = goal['x'] - prev['x']
                dy = goal['y'] - prev['y']

            target_yaw = np.arctan2(dy, dx)
            angle_diff = self.normalize_angle(target_yaw - self.yaw)
            turn_time = abs(angle_diff) / self.angular_speed

            if elapsed < turn_time:
                vel.angular.z = self.angular_speed if angle_diff > 0 else -self.angular_speed
                self.cmd_pub.publish(vel)
            else:
                self.cmd_pub.publish(Twist())
                self.yaw = target_yaw  # simular cambio de orientación
                self.state = 'FORWARD'
                self.start_time = now

                if goal['time'] > 0:
                    self.phase_duration = goal['time']
                    self.current_speed = self.linear_speed_default
                else:
                    if self.current_index == 0:
                        x0, y0 = 0.0, 0.0
                    else:
                        prev = self.goals[self.current_index - 1]
                        x0, y0 = prev['x'], prev['y']
                    dx = goal['x'] - x0
                    dy = goal['y'] - y0
                    distance = np.sqrt(dx**2 + dy**2)
                    self.phase_duration = distance / goal['speed']
                    self.current_speed = goal['speed']

        elif self.state == 'FORWARD':
            if elapsed < self.phase_duration:
                vel.linear.x = self.current_speed
                self.cmd_pub.publish(vel)
            else:
                self.cmd_pub.publish(Twist())
                self.get_logger().info(f"✅ Punto {self.current_index + 1} completado.")
                self.current_index += 1
                if self.current_index < len(self.goals):
                    self.state = 'TURN'
                    self.start_time = time.time()
                else:
                    self.state = 'IDLE'
                    self.get_logger().info("🏁 Ruta completada.")

    def normalize_angle(self, angle):
        return np.arctan2(np.sin(angle), np.cos(angle))  # compacta y limpia

def main(args=None):
    rclpy.init(args=args)
    node = PathController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
