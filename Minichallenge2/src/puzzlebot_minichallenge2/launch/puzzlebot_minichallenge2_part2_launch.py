from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Obtener la ruta del paquete y del archivo YAML
    pkg_dir = get_package_share_directory('puzzlebot_minichallenge2')
    config_path = os.path.join(pkg_dir, 'config', 'path.yaml')

    # Nodo Path Generator
    PathGenerator = Node(
        name="PathGenerator",
        package='puzzlebot_minichallenge2',
        executable='PathGenerator',  # nombre definido en entry_points
        emulate_tty=True,
        output='screen',
        parameters=[
            {'use_sim_time': False},
            {'config_file': config_path}
        ]
    )

    # Nodo Controller
    PathController = Node(
        name="PathController",
        package='puzzlebot_minichallenge2',
        executable='PathController',
        emulate_tty=True,
        output='screen',
        parameters=[
            {'use_sim_time': False}
        ]
    )

    # Herramientas visuales (opcional)
    rqt_graph = Node(
        name='rqt_graph',
        package='rqt_graph',
        executable='rqt_graph'
    )

    # Launch Description
    l_d = LaunchDescription([PathGenerator, PathController, rqt_graph])

    return l_d
