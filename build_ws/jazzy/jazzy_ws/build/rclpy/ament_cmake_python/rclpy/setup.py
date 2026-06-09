from setuptools import find_packages
from setuptools import setup

setup(
    name='rclpy',
    version='7.1.10',
    packages=find_packages(
        include=('rclpy', 'rclpy.*')),
)
