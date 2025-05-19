# ------------------------------------------------------------------------------
# Proyecto: Mini Challenge 4 - launch file
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

def generate_launch_description():

    # Nodo que sigue la línea usando control PD
    Controller = Node(
        name="Controller",
        package='puzzlebot_minichallenge5',
        executable='Controller',
        emulate_tty=True,
        output='screen'
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

    # Nodo que lanza la interfaz gráfica de rqt_image_view
    rqt_image_view = Node(
        name="rqt_image_view",
        package='rqt_image_view',
        executable='rqt_image_view',
        emulate_tty=True,
        output='screen',
        parameters=[
            {'/processed_line_image'},  # Tópico de imagen procesada
        ]
    )


    # Descripción de lanzamiento (solo con nodos activos por defecto)
    l_d = LaunchDescription([LineFollower, Controller, rqt_image_view])

    return l_d
