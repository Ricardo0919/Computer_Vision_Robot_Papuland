# ------------------------------------------------------------------------------
# Proyecto: Mini Challenge 4 - setup.py
# Materia: Implementación de Robótica Inteligente
# Fecha: 22 de abril de 2025
# Alumnos:
#   - Jonathan Arles Guevara Molina  | A01710380
#   - Ezzat Alzahouri Campos         | A01710709
#   - José Ángel Huerta Ríos         | A01710607
#   - Ricardo Sierra Roa             | A01709887
# ------------------------------------------------------------------------------

from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'puzzlebot_minichallenge4'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.[yma]*'))),
    ],
    install_requires=[
        'setuptools',
        'transforms3d',
    ],
    zip_safe=True,
    maintainer='ricardosierra',
    maintainer_email='rickisierra03@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'Controller = puzzlebot_minichallenge4.Controller:main',
            'OdometryNode = puzzlebot_minichallenge4.OdometryNode:main',
            'PathGenerator = puzzlebot_minichallenge4.PathGenerator:main',
            'PathController = puzzlebot_minichallenge4.PathController:main',
            'TrafficLightDetector = puzzlebot_minichallenge4.TrafficLightDetector:main',
        ],
    },
)
