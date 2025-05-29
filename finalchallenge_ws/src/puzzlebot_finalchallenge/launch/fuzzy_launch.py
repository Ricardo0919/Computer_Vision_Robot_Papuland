# ------------------------------------------------------------------------------
# Proyecto: Navegación de Línea con Control Difuso Basado en Centroide - Launch file
# Materia: Implementación de Robótica Inteligente
# Fecha: 6 de junio de 2025
# Alumnos:
#   - Jonathan Arles Guevara Molina  | A01710380
#   - Ezzat Alzahouri Campos         | A01710709
#   - José Ángel Huerta Ríos         | A01710607
#   - Ricardo Sierra Roa             | A01709887
# ------------------------------------------------------------------------------

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # Nodo controlador en base a seguidor de línea y detección de colores de semáforo
    Controller = Node(
        name="ControllerFuzzy",
        package='puzzlebot_finalchallenge',
        executable='ControllerFuzzy', 
        emulate_tty=True,
        output='screen',
        parameters=[
            {'v_max': 0.18} # Límite de máxima velocidad del robot
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
            {'roi_left': 15},   # Margen izquierdo del área de interés
            {'roi_right': 15}   # Margen derecho del área de interés
        ]
    )

    # Nodo que lanza la interfaz gráfica de rqt_image_view para visualizar la imagen de lineas detectadas
    rqt_image_view_line = Node(
        name="rqt_image_view",
        package='rqt_image_view',
        executable='rqt_image_view',
        emulate_tty=True,
        output='screen',
        arguments=['/processed_line_image'] # Tópico de imagen procesada
    )

    # Descripción de lanzamiento (solo con nodos activos por defecto)
    l_d = LaunchDescription([LineFollower, Controller, rqt_image_view_line])

    return l_d
