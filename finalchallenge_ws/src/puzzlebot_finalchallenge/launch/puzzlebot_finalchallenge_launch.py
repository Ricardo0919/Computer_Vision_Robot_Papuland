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

def generate_launch_description():

    # Nodo controlador en base a seguidor de línea y detección de colores de semáforo
    Controller = Node(
        name="Controller",
        package='puzzlebot_finalchallenge',
        executable='Controller',
        emulate_tty=True,
        output='screen',
        parameters=[
            {'v_max': 0.2} # Límite de máxima velocidad del robot
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
            {'roi_ratio': 20}  # Proporción del área de interés respecto a la imagen
        ]
    )

    # Nodo que detecta colores de semáforo
    TrafficLightDetector = Node(
        name="TrafficLightDetector",
        package='puzzlebot_finalchallenge',
        executable='TrafficLightDetector',
        emulate_tty=True,
        output='screen',
        parameters=[
            {'mode': 'real'},        # Modo de operación: 'sim' o 'real'
            {'max_pixels': 50},      # Área mínima para detectar un objeto
            {'max_area': 200},       # Área máxima para detectar un objeto
            {'roi_ratio': 130},      # Proporción del área de interés respecto a la imagen
            {'roi_left': 30},        # Última columna ROI-izq (0-159)]
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
