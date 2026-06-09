from setuptools import find_packages
from setuptools import setup

setup(
    name='rosidl_typesupport_fastrtps_cpp',
    version='3.6.3',
    packages=find_packages(
        include=('rosidl_typesupport_fastrtps_cpp', 'rosidl_typesupport_fastrtps_cpp.*')),
)
