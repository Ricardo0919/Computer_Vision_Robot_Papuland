# ------------------------------------------------------------------------------
# Proyecto: Puzzlebot Final Challenge - Launch file para la computadora del proyecto final 
# Materia: Implementación de Robótica Inteligente
# Fecha: 14 de junio de 2025
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

    # Nodo que lanza la interfaz gráfica de rqt_image_view para visualizar la imagen de linea detectada a seguir
    rqt_image_view_line = Node(
        name="rqt_image_view",
        package='rqt_image_view',
        executable='rqt_image_view',
        emulate_tty=True,
        output='screen',
        arguments=['/line_follower'] # Tópico de imagen procesada line follower
    )

    # Nodo que lanza la interfaz gráfica de rqt_image_view para visualizar la imagen de colores detectados
    rqt_image_view_color = Node(
        name="rqt_image_view",
        package='rqt_image_view',
        executable='rqt_image_view',
        emulate_tty=True,
        output='screen',
        arguments=['/traffic_light'] # Tópico de imagen procesada traffic light
    )

    # Nodo que lanza la interfaz gráfica de rqt_image_view para visualizar la imagen de señales de tráfico detectadas
    rqt_image_view_signal = Node(
        name="rqt_image_view",
        package='rqt_image_view',
        executable='rqt_image_view',
        emulate_tty=True,
        output='screen',
        arguments=['/traffic_signal']  # Tópico de imagen procesada traffic signal
    )

    # Nodo que lanza la interfaz gráfica de rqt_image_view para visualizar la imagen de cruces detectados
    rqt_image_view_zebra = Node(
        name="rqt_image_view",
        package='rqt_image_view',
        executable='rqt_image_view',
        emulate_tty=True,
        output='screen',
        arguments=['/zebra_image']  # Tópico de imagen procesada zebra
    )

    # Descripción de lanzamiento
    l_d = LaunchDescription([Controller, TrafficSignalDetector, TrafficLightDetector, OdometryNode, PathFollower, rqt_image_view_signal, rqt_image_view_color, rqt_image_view_line, rqt_image_view_zebra])

    return l_d
