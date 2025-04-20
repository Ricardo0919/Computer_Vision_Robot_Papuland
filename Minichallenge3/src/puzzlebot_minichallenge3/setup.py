from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'puzzlebot_minichallenge3'

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
            'Controller = puzzlebot_minichallenge3.Controller:main',
            'OrientationComparator = puzzlebot_minichallenge3.OrientationComparator:main',
            'OdometryNode = puzzlebot_minichallenge3.OdometryNode:main',
            'PathGenerator = puzzlebot_minichallenge3.PathGenerator:main',
            'PathController = puzzlebot_minichallenge3.PathController:main',
        ],
    },
)
