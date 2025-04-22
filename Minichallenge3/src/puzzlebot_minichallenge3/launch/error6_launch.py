from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('puzzlebot_minichallenge3')
    error_path = os.path.join(pkg_dir, 'config', 'error_groups.yaml')

    OdometryNodeSim = Node(
        name='OdometryNode',
        package='puzzlebot_minichallenge3',
        executable='OdometryNode',
        emulate_tty=True,
        output='screen',
        parameters=[
            {'angular_correction_factor': 0.90},
            {'linear_correction_factor': 0.92}
        ]
    )

    OdometryNodeReal = Node(
        name='OdometryNode',
        package='puzzlebot_minichallenge3',
        executable='OdometryNode',
        emulate_tty=True,
        output='screen',
        parameters=[
            {'angular_correction_factor': 1.02},
            {'linear_correction_factor': 0.94}
        ]
    )

    PathGeneratorErrors = Node(
        name="PathGeneratorErrors",
        package='puzzlebot_minichallenge3',
        executable='PathGeneratorErrors',
        emulate_tty=True,
        output='screen',
        parameters=[
            {'path_file': error_path},
            {'group_name': 'error_6'}
        ]
    )

    PathController = Node(
        name="PathController",
        package='puzzlebot_minichallenge3',
        executable='PathController',
        emulate_tty=True,
        output='screen'
    )

    return LaunchDescription([OdometryNodeReal, PathGeneratorErrors, PathController])

