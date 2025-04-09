from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    ControllerOdometry = Node(
        name="ControllerOdometry",
        package='puzzlebot_minichallenge2',
        executable='ControllerOdometry',
        emulate_tty=True,
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'mode': 'time'},
            {'total_time': 90.0},
            {'side_length': 2.0}
        ]  
    )
    
    # Nodo para mostrar el grafo de nodos de ROS en rqt_graph
    rqt_graph = Node(
        name='rqt_graph',
        package='rqt_graph',
        executable='rqt_graph'
    )

    # Definir la descripción de lanzamiento con todos los nodos configurados
    l_d = LaunchDescription([ControllerOdometry, rqt_graph])

    return l_d  # Retornar la descripción de lanzamiento