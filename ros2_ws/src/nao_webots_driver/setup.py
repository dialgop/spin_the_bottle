import os
from glob import glob

from setuptools import setup

package_name = 'nao_webots_driver'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.wbt')),
        (os.path.join('share', package_name, 'resource'),
         ['resource/nao_webots.urdf', 'resource/ros2_control.yml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Diego Alejandro Gomez Pardo',
    maintainer_email='dialgop@gmail.com',
    description='ROS2 / Webots bridge for a simulated NAO, replacing the physical robot used by the original 2015 spin_the_bottle project.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
