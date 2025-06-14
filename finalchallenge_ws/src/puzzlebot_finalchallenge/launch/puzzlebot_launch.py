# ------------------------------------------------------------------------------
# Proyecto: Puzzlebot Final Challenge - Launch file para el puzzlebot del proyecto final 
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

def generate_launch_description():

    # Nodo que detecta la línea a seguir
    LineFollower = Node(
        name="LineFollower",
        package='puzzlebot_finalchallenge',
        executable='LineFollower',
        emulate_tty=True,
        output='screen',
        parameters=[
            {'mode': 'real'},   # Modo de operación: 'sim' o 'real'
            {'roi_ratio': 15},  # Proporción del área de interés respecto a la imagen
            {'roi_left': 0},    # Margen izquierdo del área de interés
            {'roi_right': 0}    # Margen derecho del área de interés
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
            {'roi_bottom': 110},  # Margen inferior del área de interés
            {'roi_left': 0},      # Margen izquierdo del área de interés
            {'roi_right': 0}      # Margen derecho del área de interés
        ]
    )

    # Descripción de lanzamiento 
    l_d = LaunchDescription([LineFollower, ZebraDetection])

    return l_d
