import rclpy
from rclpy.node import Node
from custom_interfaces.msg import PathPose
from geometry_msgs.msg import Point, Quaternion
import os
import yaml
from ament_index_python.packages import get_package_share_directory

class PathGenerator(Node):
    def __init__(self):
        super().__init__('PathGenerator')

        self.publisher_ = self.create_publisher(PathPose, '/goals', 10)

        config_path = os.path.join(
            get_package_share_directory('puzzlebot_minichallenge3'),
            'config', 'path.yaml')

        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)

        self.waypoints = config.get('waypoints', [])

        if not self.waypoints or (self.waypoints[0]['x'] != 0.0 or self.waypoints[0]['y'] != 0.0):
            self.get_logger().info("Insertando (0,0) al inicio por defecto.")
            self.waypoints.insert(0, {'x': 0.0, 'y': 0.0})

        self.publish_path()

    def publish_path(self):
        for i, wp in enumerate(self.waypoints):
            msg = PathPose()
            msg.pose.position = Point(x=wp['x'], y=wp['y'], z=0.0)
            msg.pose.orientation = Quaternion(w=1.0)  # No se usa orientación por ahora
            self.publisher_.publish(msg)
            self.get_logger().info(f"📍 Punto {i+1} enviado: ({wp['x']:.2f}, {wp['y']:.2f})")


def main(args=None):
    rclpy.init(args=args)
    node = PathGenerator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()