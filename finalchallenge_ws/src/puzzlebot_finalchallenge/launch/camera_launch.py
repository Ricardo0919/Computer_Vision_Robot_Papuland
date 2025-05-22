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

    # Nodo que detecta la línea a seguir
    LineFollower = Node(
        name="LineFollower",
        package='puzzlebot_finalchallenge',
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
    l_d = LaunchDescription([LineFollower, rqt_image_view])

    return l_d
