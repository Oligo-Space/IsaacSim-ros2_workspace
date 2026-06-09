from setuptools import find_packages
from setuptools import setup

setup(
    name='rosidl_typesupport_cpp',
    version='3.2.2',
    packages=find_packages(
        include=('rosidl_typesupport_cpp', 'rosidl_typesupport_cpp.*')),
)
