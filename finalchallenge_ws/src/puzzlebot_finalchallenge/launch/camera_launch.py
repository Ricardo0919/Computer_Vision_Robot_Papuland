# ------------------------------------------------------------------------------
# Proyecto: Puzzlebot Final Challenge - Launch File para hacer debug de nodos respecto a la cámara
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
            {'roi_left': 20},     # Margen izquierdo del área de interés
            {'roi_right': 20}     # Margen derecho del área de interés
        ]
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

    # Nodo que lanza la interfaz gráfica de rqt_image_view para visualizar la imagen de zebra detectada
    rqt_image_view_zebra = Node(
        name="rqt_image_view",
        package='rqt_image_view',
        executable='rqt_image_view',
        emulate_tty=True,
        output='screen',
        arguments=['/zebra_image'] # Tópico de imagen procesada 
    )

    # Descripción de lanzamiento
    l_d = LaunchDescription([TrafficLightDetector, rqt_image_view_signal])

    return l_d
