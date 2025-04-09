from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory  # Añadir esto
import os  # Añadir esto

def generate_launch_description():
    # Obtener la ruta del paquete y del archivo YAML
    pkg_dir = get_package_share_directory('puzzlebot_minichallenge2')
    config_path = os.path.join(pkg_dir, 'config', 'path_config.yaml')  # Definir ruta absoluta

    # Nodo PathGenerator (corregir nombre del ejecutable y parámetro)
    PathGenerator = Node(
        name="path_generator",
        package='puzzlebot_minichallenge2',
        executable='PathGenerator',  # Nombre debe coincidir con entry_points
        emulate_tty=True,
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'config_file': config_path}  # Usar la variable definida
        ]
    )
    # Nodo 1 generador de puntos de referencia para el motor
    Controller = Node(
        name="Controller",
        package='puzzlebot_minichallenge2',
        executable='Controller',
        emulate_tty=True,
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'mode': 'time'},
            {'total_time': 30.0},
            {'side_length': 2.0}
        ]
        
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