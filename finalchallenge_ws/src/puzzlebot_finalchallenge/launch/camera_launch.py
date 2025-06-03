# ------------------------------------------------------------------------------
# Proyecto: Puzzlebot Final Challenge - Semaforo launch file
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

    # Nodo que detecta colores de semáforo
    TrafficLightDetector = Node(
        name="TrafficLightDetector",
        package='puzzlebot_finalchallenge',
        executable='TrafficLightDetector',
        emulate_tty=True,
        output='screen',
    )

    # Nodo que detecta colores de semáforo
    ZebraDetection = Node(
        name="ZebraDetection",
        package='puzzlebot_finalchallenge',
        executable='ZebraDetection',
        emulate_tty=True,
        output='screen',
        parameters=[
            {'mode': 'real'},     # Modo de operación: 'sim' o 'real'
            {'roi_top': 50},      # Margen superior del área de interés
            {'roi_bottom': 100},  # Margen inferior del área de interés
            {'roi_left': 20},     # Margen izquierdo del área de interés
            {'roi_right': 20}     # Margen derecho del área de interés
        ]
    )

    # Nodo que lanza la interfaz gráfica de rqt_image_view para visualizar la imagen de colores detectados
    rqt_image_view_color = Node(
        name="rqt_image_view",
        package='rqt_image_view',
        executable='rqt_image_view',
        emulate_tty=True,
        output='screen',
        #arguments=['/processed_img']  # Tópico de imagen procesada traffic light
        arguments=['/zebra_image']     # Tópico de imagen procesada zebra
    )

    # Descripción de lanzamiento (solo con nodos activos por defecto)
    l_d = LaunchDescription([ZebraDetection, rqt_image_view_color])

    return l_d
