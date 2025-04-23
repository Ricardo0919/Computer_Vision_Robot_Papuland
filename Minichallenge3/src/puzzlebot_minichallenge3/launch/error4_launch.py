# ------------------------------------------------------------------------------
# Proyecto: Mini Challenge 3
# Materia: Implementación de Robótica Inteligente
# Fecha: 22 de abril de 2025
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

    # Obtener la ruta del archivo de configuración con los grupos de errores
    pkg_dir = get_package_share_directory('puzzlebot_minichallenge3')
    error_path = os.path.join(pkg_dir, 'config', 'error_groups.yaml')

    # Nodo de odometría para entorno real (con parámetros calibrados)
    OdometryNodeReal = Node(
        name='OdometryNode',
        package='puzzlebot_minichallenge3',
        executable='OdometryNode',
        emulate_tty=True,
        output='screen',
        parameters=[
            {'angular_correction_factor': 1.02},  # Ajuste de giros para entorno real
            {'linear_correction_factor': 0.94}    # Ajuste de avance lineal real
        ]
    )

    # Nodo alternativo (no lanzado por defecto) para simulación
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

    # Nodo que publica grupos de errores predefinidos desde el archivo YAML
    PathGeneratorErrors = Node(
        name="PathGeneratorErrors",
        package='puzzlebot_minichallenge3',
        executable='PathGeneratorErrors',
        emulate_tty=True,
        output='screen',
        parameters=[
            {'path_file': error_path},       # Ruta del archivo con errores
            {'group_name': 'error_4'}        # Grupo específico de errores a simular
        ]
    )

    # Nodo controlador que sigue la trayectoria con control PID
    PathController = Node(
        name="PathController",
        package='puzzlebot_minichallenge3',
        executable='PathController',
        emulate_tty=True,
        output='screen'
    )

    # Descripción del lanzamiento con nodos enfocados en análisis de errores
    return LaunchDescription([OdometryNodeReal, PathGeneratorErrors, PathController])
