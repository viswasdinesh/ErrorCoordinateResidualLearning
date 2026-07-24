from setuptools import find_packages
from setuptools import setup

setup(
    name='ommpc_ros2',
    version='0.0.0',
    packages=find_packages(
        include=('ommpc_ros2', 'ommpc_ros2.*')),
)
