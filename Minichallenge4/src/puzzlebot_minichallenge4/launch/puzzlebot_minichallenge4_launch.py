# ------------------------------------------------------------------------------
# Proyecto: Mini Challenge 4 - launch.py
# Materia: Implementación de Robótica Inteligente
# Fecha: 14 de mayo de 2025
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
    pkg_dir = get_package_share_directory('puzzlebot_minichallenge4')
    config_path = os.path.join(pkg_dir, 'config', 'path.yaml')

    # Nodo de odometría para simulación (definido pero no se lanza por defecto)
    OdometryNodeSim = Node(
        name='OdometryNode',
        package='puzzlebot_minichallenge4',
        executable='OdometryNode',
        emulate_tty=True,
        output='screen',
        parameters=[
            {'angular_correction_factor': 0.90},  # Corrección angular en simulación
            {'linear_correction_factor': 0.92}    # Corrección lineal en simulación
        ]
    )

    # Nodo de odometría para el robot real (activo por defecto)
    OdometryNodeReal = Node(
        name='OdometryNode',
        package='puzzlebot_minichallenge4',
        executable='OdometryNode',
        emulate_tty=True,
        output='screen',
        parameters=[
            {'angular_correction_factor': 1.02},  # Ajuste fino de giros
            {'linear_correction_factor': 0.94}    # Ajuste fino de avance lineal
        ]
    )

    # Nodo que genera los puntos de trayectoria a partir del archivo YAML
    PathGenerator = Node(
        name="PathGenerator",
        package='puzzlebot_minichallenge4',
        executable='PathGenerator',
        emulate_tty=True,
        output='screen',
        parameters=[
            {'path_file': config_path}  # Ruta del archivo con los waypoints
        ]
    )

    # Nodo que sigue la trayectoria usando control PD
    PathController = Node(
        name="PathController",
        package='puzzlebot_minichallenge4',
        executable='PathController',
        emulate_tty=True,
        output='screen'
    )

    # Descripción de lanzamiento (solo con nodos activos por defecto)
    l_d = LaunchDescription([OdometryNodeSim, PathGenerator, PathController])

    return l_d
