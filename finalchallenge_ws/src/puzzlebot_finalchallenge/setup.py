# ------------------------------------------------------------------------------
# Proyecto: Puzzlebot Final Challenge - setup.py
# Materia: Implementación de Robótica Inteligente
# Fecha: 14 de junio de 2025
# Alumnos:
#   - Jonathan Arles Guevara Molina  | A01710380
#   - Ezzat Alzahouri Campos         | A01710709
#   - José Ángel Huerta Ríos         | A01710607
#   - Ricardo Sierra Roa             | A01709887
# ------------------------------------------------------------------------------

from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'puzzlebot_finalchallenge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'models'), glob(os.path.join(package_name, 'models', '*.pt'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.[yma]*'))),
    ],
    install_requires=[
        'setuptools',
        'ament_index_python',
        'rclpy',
        'numpy',
        'ultralytics',
        'opencv-python',
        'transforms3d',
        'cv_bridge'
    ],

    zip_safe=True,
    maintainer='ricardosierra',
    maintainer_email='rickisierra03@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ControllerPD = puzzlebot_finalchallenge.ControllerPD:main',
            'LineFollower = puzzlebot_finalchallenge.LineFollower:main',
            'OdometryNode = puzzlebot_finalchallenge.OdometryNode:main',
            'PathFollower = puzzlebot_finalchallenge.PathFollower:main',
            'ZebraDetection = puzzlebot_finalchallenge.ZebraDetection:main',
            'TrafficLightDetector = puzzlebot_finalchallenge.TrafficLightDetector:main',
            'TrafficSignalDetector = puzzlebot_finalchallenge.TrafficSignalDetector:main',
            'TrafficLightImageTaker = puzzlebot_finalchallenge.TrafficLightImageTaker:main',
            'TrafficSignalImageTaker = puzzlebot_finalchallenge.TrafficSignalImageTaker:main',
        ],
    },
)
