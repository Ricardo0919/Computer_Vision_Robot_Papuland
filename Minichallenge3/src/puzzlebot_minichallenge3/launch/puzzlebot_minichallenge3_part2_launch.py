from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Obtener la ruta del paquete y del archivo YAML
    pkg_dir = get_package_share_directory('puzzlebot_minichallenge3')
    config_path = os.path.join(pkg_dir, 'config', 'path.yaml')

    # Nodo de Odometría con parámetros ajustables
    OdometryNodeSim = Node(
        name='OdometryNode',
        package='puzzlebot_minichallenge3',
        executable='OdometryNode',
        emulate_tty=True,
        output='screen',
        parameters=[
            {'angular_correction_factor': 0.90},
            {'linear_correction_factor': 0.92}
        ]
    )

    # Nodo de Odometría con parámetros ajustables
    OdometryNodeReal = Node(
        name='OdometryNode',
        package='puzzlebot_minichallenge3',
        executable='OdometryNode',
        emulate_tty=True,
        output='screen',
        parameters=[
            {'angular_correction_factor': 0.955},
            {'linear_correction_factor': 0.94}
        ]
    )

    # Nodo PathGenerator (publica la trayectoria)
    PathGenerator = Node(
        name="PathGenerator",
        package='puzzlebot_minichallenge3',
        executable='PathGenerator',
        emulate_tty=True,
        output='screen',
        parameters=[
            {'path_file': config_path}
        ]
    )

    # Nodo PathController (sigue los puntos publicados)
    PathController = Node(
        name="PathController",
        package='puzzlebot_minichallenge3',
        executable='PathController',
        emulate_tty=True,
        output='screen'
    )

    # Herramientas visuales (opcional)
    rqt_graph = Node(
        name='rqt_graph',
        package='rqt_graph',
        executable='rqt_graph'
    )

    # Launch Description
    l_d = LaunchDescription([OdometryNodeReal, PathGenerator, PathController])

    return l_d
