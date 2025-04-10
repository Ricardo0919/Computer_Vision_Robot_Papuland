import rclpy
from rclpy.node import Node
from custom_interfaces.msg import PathPose
from geometry_msgs.msg import Point, Quaternion
import os
import yaml
from ament_index_python.packages import get_package_share_directory
import math

class PathGenerator(Node):
    def __init__(self):
        super().__init__('path_generator')

        self.publisher_ = self.create_publisher(PathPose, '/pose', 10)

        config_path = os.path.join(
            get_package_share_directory('puzzlebot_minichallenge2'),
            'config',
            'path.yaml'
        )

        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)

        self.path_type = config.get('path_type', 'time')
        self.target = float(config.get('target', 10.0))
        self.waypoints = config.get('waypoints', [])

        # Límites físicos del robot
        self.max_linear_speed = 0.3  # m/s
        self.max_angular_speed = 0.6  # rad/s

        if not self.waypoints or (self.waypoints[0]['x'] != 0.0 or self.waypoints[0]['y'] != 0.0):
            self.get_logger().info("Insertando (0,0) al inicio por defecto.")
            self.waypoints.insert(0, {'x': 0.0, 'y': 0.0})

        self.get_logger().info(f"🔁 Generando trayectoria en modo: {self.path_type}")
        self.validate_and_publish_path()

    def validate_and_publish_path(self):
        total_distance = 0.0
        segments = []

        if self.path_type == 'time':
            for i in range(len(self.waypoints) - 1):
                p1 = self.waypoints[i]
                p2 = self.waypoints[i+1]
                dx = p2['x'] - p1['x']
                dy = p2['y'] - p1['y']
                dist = math.hypot(dx, dy)
                total_distance += dist
                segments.append(dist)

        all_reachable = True
        msgs = []

        for i, point in enumerate(self.waypoints):
            msg = PathPose()
            msg.pose.position = Point(
                x=float(point['x']),
                y=float(point['y']),
                z=0.0
            )
            msg.pose.orientation = Quaternion(w=1.0)
            is_reachable = True

            if self.path_type == 'time':
                msg.time_to_reach = self.target if i == 0 else 0.0
                msg.linear_velocity = 0.0

                if i > 0:
                    dist = segments[i - 1]
                    seg_time = self.target * (dist / total_distance)
                    est_speed = dist / seg_time if seg_time > 0 else 0.0

                    if est_speed > self.max_linear_speed:
                        is_reachable = False
                        self.get_logger().warn(f"⚠️ Punto {i+1} requiere {est_speed:.2f} m/s, excede {self.max_linear_speed:.2f} m/s")

            else:  # speed mode
                msg.time_to_reach = 0.0
                msg.linear_velocity = self.target if i == 0 else 0.0

                if self.target > self.max_linear_speed:
                    is_reachable = False
                    if i == 0:
                        self.get_logger().warn(f"⚠️ Velocidad definida por el usuario = {self.target:.2f} m/s excede el límite de {self.max_linear_speed:.2f} m/s")

            msg.angular_velocity = 0.0
            msg.is_reachable = is_reachable
            msgs.append(msg)

            estado = "✅ alcanzable" if is_reachable else "❌ NO alcanzable"
            self.get_logger().info(f"Punto {i+1}: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f}) → {estado}")

            if not is_reachable:
                all_reachable = False

        if all_reachable:
            self.get_logger().info("✅ Todos los puntos son alcanzables. Publicando trayectoria...")
            for msg in msgs:
                self.publisher_.publish(msg)
        else:
            self.get_logger().error("❌ No se puede ejecutar la trayectoria: hay puntos no alcanzables. Revisa los parámetros del YAML.")

def main(args=None):
    rclpy.init(args=args)
    node = PathGenerator()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
