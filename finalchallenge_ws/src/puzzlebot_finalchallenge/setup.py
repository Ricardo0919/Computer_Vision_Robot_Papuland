# ------------------------------------------------------------------------------
# Proyecto: Mini Challenge 5 - setup.py
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
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.[yma]*'))),
        (os.path.join('share', package_name, 'models'), glob(os.path.join(package_name, 'models', '*.onnx'))),
    ],
    install_requires=[
        'setuptools',
        'ament_index_python',
        'onnxruntime',
        'opencv-python',
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
            'TrafficLightImageTaker = puzzlebot_finalchallenge.TrafficLightImageTaker:main',
            'color_filter = puzzlebot_finalchallenge.color_filter:main',
            'Controller = puzzlebot_finalchallenge.Controller:main',
            'LineFollower = puzzlebot_finalchallenge.LineFollower:main',
            'TrafficLightDetector = puzzlebot_finalchallenge.TrafficLightDetector:main',
        ],
    },
)
