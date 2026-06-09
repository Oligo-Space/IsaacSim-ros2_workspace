from setuptools import find_packages
from setuptools import setup

setup(
    name='rosidl_generator_py',
    version='0.22.2',
    packages=find_packages(
        include=('rosidl_generator_py', 'rosidl_generator_py.*')),
)
