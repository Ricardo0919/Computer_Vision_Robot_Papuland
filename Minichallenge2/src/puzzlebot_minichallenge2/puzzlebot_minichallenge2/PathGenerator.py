import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from custom_interfaces.msg import PathPose
import yaml
import os
from ament_index_python.packages import get_package_share_directory

class PathGenerator(Node):
    def __init__(self):
        super().__init__('path_generator')

        self.publisher_ = self.create_publisher(PathPose, '/pose', 10)

        config_file = os.path.join(
            get_package_share_directory('puzzlebot_minichallenge2'),
            'config',
            'path.yaml'
        )

        with open(config_file, 'r') as f:
            config = yaml.safe_load(f)

        self.path_type = config.get('path_type', 'time')
        self.target = config.get('target', 5.0)
        self.waypoints = config.get('waypoints', [])

        self.publish_path()

    def publish_path(self):
        for i, point in enumerate(self.waypoints):
            msg = PathPose()
            msg.pose.position.x = float(point['x'])
            msg.pose.position.y = float(point['y'])
            msg.pose.position.z = 0.0
            msg.pose.orientation.w = 1.0  # orientación por defecto

            if self.path_type == 'time':
                msg.time_to_reach = float(self.target)
                msg.linear_velocity = 0.0
                msg.angular_velocity = 0.0
            elif self.path_type == 'speed':
                msg.time_to_reach = 0.0
                msg.linear_velocity = float(self.target)
                msg.angular_velocity = 0.0

            msg.is_reachable = True  # puedes calcular esto más adelante si quieres

            self.get_logger().info(f'Publicando punto {i+1}: {msg}')
            self.publisher_.publish(msg)
            self.create_timer(1.0, lambda: None)  # simula una pausa

def main(args=None):
    rclpy.init(args=args)
    node = PathGenerator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
