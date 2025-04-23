# ------------------------------------------------------------------------------
# Proyecto: Mini Challenge 3
# Materia: Implementación de Robótica Inteligente
# Fecha: 22 de abril de 2025
# Alumnos:
#   - Jonathan Arles Guevara Molina  | A01710380
#   - Ezzat Alzahouri Campos         | A01710709
#   - José Ángel Huerta Ríos         | A01710607
#   - Ricardo Sierra Roa             | A01709887
# ------------------------------------------------------------------------------

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # Nodo de odometría para simulación
    # Configurado con factores de corrección adaptados a condiciones simuladas
    OdometryNodeSim = Node(
        package='puzzlebot_minichallenge3',
        executable='OdometryNode', 
        name='OdometryNode',
        output='screen',
        emulate_tty=True,
        parameters=[
            {'angular_correction_factor': 0.90},  # Factor de corrección angular para simulador
            {'linear_correction_factor': 0.92}    # Factor de corrección lineal para simulador
        ]
    )

    # Nodo de odometría para robot real
    # Ajustado con factores específicos calibrados para minimizar error de desplazamiento real
    OdometryNodeReal = Node(
        package='puzzlebot_minichallenge3',
        executable='OdometryNode', 
        name='OdometryNode',
        output='screen',
        emulate_tty=True,
        parameters=[
            {'angular_correction_factor': 0.955},  # Ajuste fino para giros en el mundo real
            {'linear_correction_factor': 0.94}     # Ajuste fino para desplazamiento recto
        ]
    )

    # Nodo del controlador PID
    # Controla el movimiento del robot en una trayectoria cuadrada de lado definido
    Controller = Node(
        package='puzzlebot_minichallenge3',
        executable='Controller',  
        name='Controller',
        output='screen',
        emulate_tty=True,
        parameters=[
            {'side_length': 2.0}  # Longitud del lado del recorrido cuadrado (en metros)
        ]
    )
    
    ld_description = LaunchDescription([OdometryNodeReal, Controller])

    return ld_description
