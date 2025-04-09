from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # Nodo 1 generador de puntos de referencia para el motor
    Controller = Node(
        name="Controller",
        package='puzzlebot_minichallenge2',
        executable='Controller',
        emulate_tty=True,
        output='screen',
        parameters=[
            {'use_sim_time': False},
            {'mode': 'time'},
            {'total_time': 90.0},
            {'side_length': 2.0}
        ]
        
    )
    
    ground_truth_node = Node(
        name="ground_truth_node",
        package='puzzlebot_minichallenge2',
        executable='ground_truth_node',
        emulate_tty=True,
        output='screen',
    )

    # Definir la descripción de lanzamiento con todos los nodos configurados
    l_d = LaunchDescription([ground_truth_node, Controller])

    return l_d  # Retornar la descripción de lanzamiento