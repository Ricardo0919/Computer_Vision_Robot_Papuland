# ------------------------------------------------------------------------------
# Proyecto: Puzzlebot Final Challenge - launch file
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

    # Nodo controlador en base a seguidor de línea y detección de colores de semáforo
    Controller = Node(
        name="ControllerPD",
        package='puzzlebot_finalchallenge',
        executable='ControllerPD', 
        emulate_tty=True,
        output='screen',
        parameters=[
            {'v_max': 0.15} # Límite de máxima velocidad del robot
        ]
    )

    # Nodo que detecta la línea a seguir
    LineFollower = Node(
        name="LineFollower",
        package='puzzlebot_finalchallenge',
        executable='LineFollower',
        emulate_tty=True,
        output='screen',
        parameters=[
            {'mode': 'real'},  # Modo de operación: 'sim' o 'real'
            {'roi_ratio': 15},  # Proporción del área de interés respecto a la imagen
            {'roi_left': 0},   # Margen izquierdo del área de interés
            {'roi_right': 0}   # Margen derecho del área de interés
        ]
    )

    # Nodo que detecta si hay un cruce
    ZebraDetection = Node(
        name="ZebraDetection",
        package='puzzlebot_finalchallenge',
        executable='ZebraDetection',
        emulate_tty=True,
        output='screen',
        parameters=[
            {'mode': 'real'},     # Modo de operación: 'sim' o 'real'
            {'roi_top': 45},      # Margen superior del área de interés
            {'roi_bottom': 110},   # Margen inferior del área de interés
            {'roi_left': 0},     # Margen izquierdo del área de interés
            {'roi_right': 0}     # Margen derecho del área de interés
        ]
    )

    # Nodo que detecta colores de semáforo
    TrafficLightDetector = Node(
        name="TrafficLightDetector",
        package='puzzlebot_finalchallenge',
        executable='TrafficLightDetector',
        emulate_tty=True,
        output='screen',
    )

    # Nodo que detecta las señales de tráfico
    TrafficSignalDetector = Node(
        name="TrafficSignalDetector",
        package='puzzlebot_finalchallenge',
        executable='TrafficSignalDetector',
        emulate_tty=True,
        output='screen',
    )

    # Nodo que sigue una trayectoria predefinida
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

    # Nodo de odometría
    OdometryNode = Node(
        name="OdometryNode",
        package='puzzlebot_finalchallenge',
        executable='OdometryNode',
        emulate_tty=True,
        output='screen',
    )

    # Nodo que lanza la interfaz gráfica de rqt_image_view para visualizar la imagen de lineas detectadas
    rqt_image_view_signal = Node(
        name="rqt_image_view",
        package='rqt_image_view',
        executable='rqt_image_view',
        emulate_tty=True,
        output='screen',
        arguments=['/traffic_signal'] # Tópico de imagen procesada
    )

    # Nodo que lanza la interfaz gráfica de rqt_image_view para visualizar la imagen de colores detectados
    rqt_image_view_color = Node(
        name="rqt_image_view",
        package='rqt_image_view',
        executable='rqt_image_view',
        emulate_tty=True,
        output='screen',
        arguments=['/traffic_light'] # Tópico de imagen procesada
    )

    # Descripción de lanzamiento (solo con nodos activos por defecto)
    l_d = LaunchDescription([LineFollower, Controller, ZebraDetection, TrafficSignalDetector, TrafficLightDetector, OdometryNode, PathFollower, rqt_image_view_signal, rqt_image_view_color])

    return l_d
