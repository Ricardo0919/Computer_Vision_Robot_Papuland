import rclpy
from rclpy.node import Node
from custom_interfaces.msg import PathPose
from geometry_msgs.msg import Point, Quaternion
import os
import yaml
from ament_index_python.packages import get_package_share_directory

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

        # Insertamos (0,0) si no está al inicio
        if not self.waypoints or (self.waypoints[0]['x'] != 0.0 or self.waypoints[0]['y'] != 0.0):
            self.get_logger().info("Insertando (0,0) al inicio por defecto.")
            self.waypoints.insert(0, {'x': 0.0, 'y': 0.0})

        self.get_logger().info(f"🔁 Generando trayectoria en modo: {self.path_type}")
        self.publish_path()

    def publish_path(self):
        for i, point in enumerate(self.waypoints):
            msg = PathPose()
            msg.pose.position = Point(
                x=float(point['x']),
                y=float(point['y']),
                z=0.0
            )
            msg.pose.orientation = Quaternion(w=1.0)

            # Copiamos el target, y el controlador decide qué hacer
            if self.path_type == 'time':
                msg.time_to_reach = self.target
                msg.linear_velocity = 0.0
            else:  # speed
                msg.time_to_reach = 0.0
                msg.linear_velocity = self.target

            msg.angular_velocity = 0.0
            msg.is_reachable = True

            self.get_logger().info(f"📤 Publicando punto {i+1}: {msg}")
            self.publisher_.publish(msg)
            self.create_timer(1.0, lambda: None)

def main(args=None):
    rclpy.init(args=args)
    node = PathGenerator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
