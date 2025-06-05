# ------------------------------------------------------------------------------
# Proyecto: Puzzlebot Final Challenge - Odometry launch file
# Materia: Implementación de Robótica Inteligente
# Fecha: 12 de junio de 2025
# Alumnos:
#   - Jonathan Arles Guevara Molina  | A01710380
#   - Ezzat Alzahouri Campos         | A01710709
#   - José Ángel Huerta Ríos         | A01710607
#   - Ricardo Sierra Roa             | A01709887
# ------------------------------------------------------------------------------

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    # Obtener la ruta del archivo YAML de trayectoria
    pkg_dir = get_package_share_directory('puzzlebot_finalchallenge')
    config_path = os.path.join(pkg_dir, 'config', 'paths.yaml')

    # Nodo que detecta colores de semáforo
    PathFollower = Node(
        name="PathFollower",
        package='puzzlebot_finalchallenge',
        executable='PathFollower',
        emulate_tty=True,
        output='screen',
        parameters=[
            {'path_file': config_path}  # Ruta del archivo con los waypoints
        ]
    )

    # Nodo que detecta colores de semáforo
    OdometryNode = Node(
        name="OdometryNode",
        package='puzzlebot_finalchallenge',
        executable='OdometryNode',
        emulate_tty=True,
        output='screen',
    )

    # Descripción de lanzamiento (solo con nodos activos por defecto)
    l_d = LaunchDescription([PathFollower, OdometryNode])

    return l_d
