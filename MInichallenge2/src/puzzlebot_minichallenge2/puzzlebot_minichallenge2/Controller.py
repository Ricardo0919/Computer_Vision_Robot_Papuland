import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import math

class SquareController(Node):
    def __init__(self):
        super().__init__('square_controller')

        self.declare_parameter('side_length', 2.0)
        self.declare_parameter('linear_speed', 0.2)
        self.declare_parameter('angular_speed', 0.4)

        self.get_params()

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.loop)  # 10 Hz

        self.state = 'START'
        self.count = 0
        self.t_start = time.time()

        self.get_logger().info('SquareController iniciado.')

    def get_params(self):
        self.side_length = self.get_parameter('side_length').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value

        self.t_straight = self.side_length / self.linear_speed
        self.t_turn = (math.pi / 2) / self.angular_speed

        self.get_logger().info(f"[PARAMS] lado={self.side_length}m, v={self.linear_speed}m/s, w={self.angular_speed}rad/s")

    def loop(self):
        now = time.time()
        elapsed = now - self.t_start

        # Actualizamos los parámetros en tiempo real por si los cambiaste
        self.get_params()

        twist = Twist()

        if self.state == 'START':
            self.get_logger().info('Empezando...')
            self.state = 'STRAIGHT'
            self.t_start = now

        elif self.state == 'STRAIGHT':
            if elapsed < self.t_straight:
                twist.linear.x = self.linear_speed
            else:
                self.state = 'TURN'
                self.t_start = now
                self.get_logger().info(f'Lado {self.count + 1} completado. Girando...')

        elif self.state == 'TURN':
            if elapsed < self.t_turn:
                twist.angular.z = self.angular_speed
            else:
                self.count += 1
                if self.count >= 4:
                    self.state = 'STOP'
                    self.get_logger().info('Trayectoria completa.')
                else:
                    self.state = 'STRAIGHT'
                    self.t_start = now

        elif self.state == 'STOP':
            self.publisher.publish(Twist())  # Publishes zero velocities
            rclpy.shutdown()
            return

        self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = SquareController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
