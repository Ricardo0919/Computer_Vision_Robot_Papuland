# ------------------------------------------------------------------------------
# Proyecto: Mini Challenge 4 - camera_launch.py
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


def generate_launch_description():

    # Nodo que detecta colores de semáforo
    TrafficLightDetector = Node(
        name="TrafficLightDetector",
        package='puzzlebot_minichallenge4',
        executable='TrafficLightDetector',
        emulate_tty=True,
        output='screen',
        parameters=[
            {'mode': 'sim'}  # Modo de operación: 'sim' o 'real'
        ]
    )

    # Nodo que lanza la interfaz gráfica de rqt_image_view
    rqt_image_view = Node(
        name="rqt_image_view",
        package='rqt_image_view',
        executable='rqt_image_view',
        emulate_tty=True,
        output='screen'
    )

    # Descripción de lanzamiento (solo con nodos activos por defecto)
    l_d = LaunchDescription([rqt_image_view, TrafficLightDetector])

    return l_d
