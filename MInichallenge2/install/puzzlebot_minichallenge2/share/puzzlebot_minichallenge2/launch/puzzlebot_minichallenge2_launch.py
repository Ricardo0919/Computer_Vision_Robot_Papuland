from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # Nodo 1 generador de puntos de referencia para el motor
    Controller = Node(
        name="Controller",
        package='puzzlebot_minichallenge2',
        executable='Controller',
        emulate_tty=True,
        output='screen'
    )
    
    # Nodo para mostrar el grafo de nodos de ROS en rqt_graph
    rqt_graph = Node(
        name='rqt_graph',
        package='rqt_graph',
        executable='rqt_graph'
    )

    # Nodo para configurar parámetros en tiempo real usando rqt_reconfigure
    rqt_reconfigure = Node(
        name='rqt_reconfigure',
        package='rqt_reconfigure',
        executable='rqt_reconfigure'
    )

    # Definir la descripción de lanzamiento con todos los nodos configurados
    l_d = LaunchDescription([Controller, rqt_graph, rqt_reconfigure])

    return l_d  # Retornar la descripción de lanzamiento