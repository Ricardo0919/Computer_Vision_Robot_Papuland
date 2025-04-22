from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # Nodo de odometría
    OdometryNodeSim = Node(
        package='puzzlebot_minichallenge3',
        executable='OdometryNode', 
        name='OdometryNode',
        output='screen',
        emulate_tty=True,
        parameters=[
            {'angular_correction_factor': 0.90},
            {'linear_correction_factor': 0.92}
        ]
    )

    # Nodo de odometría
    OdometryNodeReal = Node(
        package='puzzlebot_minichallenge3',
        executable='OdometryNode', 
        name='OdometryNode',
        output='screen',
        emulate_tty=True,
        parameters=[
            {'angular_correction_factor': 0.955},
            {'linear_correction_factor': 0.94}
        ]
    )

    # Nodo controlador PID
    Controller = Node(
        package='puzzlebot_minichallenge3',
        executable='Controller',  
        name='Controller',
        output='screen',
        emulate_tty=True,
        parameters=[
            {'side_length': 2.0}
        ]
    )
    
    ld_description = LaunchDescription([OdometryNodeReal, Controller])

    return ld_description
