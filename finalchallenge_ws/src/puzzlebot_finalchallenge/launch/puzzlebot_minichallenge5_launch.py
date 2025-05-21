# ------------------------------------------------------------------------------
# Proyecto: Mini Challenge 5 - launch file
# Materia: Implementación de Robótica Inteligente
# Fecha: 21 de mayo de 2025
# Alumno:
#   - Ricardo Sierra Roa             | A01709887
# ------------------------------------------------------------------------------

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # Nodo controlador en base a seguidor de línea y detección de colores de semáforo
    Controller = Node(
        name="Controller",
        package='puzzlebot_minichallenge5',
        executable='Controller',
        emulate_tty=True,
        output='screen',
        parameters=[
            {'max_speed': 0.5} # Límite de máxima velocidad del robot
        ]
    )

    # Nodo que detecta la línea a seguir
    LineFollower = Node(
        name="LineFollower",
        package='puzzlebot_minichallenge5',
        executable='LineFollower',
        emulate_tty=True,
        output='screen',
        parameters=[
            {'mode': 'sim'}  # Modo de operación: 'sim' o 'real'
        ]
    )

    # Nodo que detecta colores de semáforo
    TrafficLightDetector = Node(
        name="TrafficLightDetector",
        package='puzzlebot_minichallenge5',
        executable='TrafficLightDetector',
        emulate_tty=True,
        output='screen',
        parameters=[
            {'mode': 'sim'},  # Modo de operación: 'sim' o 'real'
            {'area': 600}     # Área mínima para detectar un objeto
        ]
    )

    # Nodo que lanza la interfaz gráfica de rqt_image_view para visualizar la imagen de lineas detectadasq
    rqt_image_view_line = Node(
        name="rqt_image_view",
        package='rqt_image_view',
        executable='rqt_image_view',
        emulate_tty=True,
        output='screen',
        parameters=[
            {'/processed_line_image'},  # Tópico de imagen procesada
        ]
    )

    # Nodo que lanza la interfaz gráfica de rqt_image_view para visualizar la imagen de colores detectados
    rqt_image_view_color = Node(
        name="rqt_image_view",
        package='rqt_image_view',
        executable='rqt_image_view',
        emulate_tty=True,
        output='screen',
        parameters=[
            {'/processed_img'},  # Tópico de imagen procesada
        ]
    )

    # Descripción de lanzamiento (solo con nodos activos por defecto)
    l_d = LaunchDescription([LineFollower, Controller, TrafficLightDetector, rqt_image_view_line, rqt_image_view_color])

    return l_d
