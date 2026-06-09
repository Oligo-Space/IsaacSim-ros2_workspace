from setuptools import find_packages
from setuptools import setup

setup(
    name='rosidl_generator_cpp',
    version='4.6.7',
    packages=find_packages(
        include=('rosidl_generator_cpp', 'rosidl_generator_cpp.*')),
)
