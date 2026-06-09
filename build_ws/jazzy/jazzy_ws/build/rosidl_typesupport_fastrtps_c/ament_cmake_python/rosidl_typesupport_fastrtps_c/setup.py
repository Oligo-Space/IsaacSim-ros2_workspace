from setuptools import find_packages
from setuptools import setup

setup(
    name='rosidl_typesupport_fastrtps_c',
    version='3.6.3',
    packages=find_packages(
        include=('rosidl_typesupport_fastrtps_c', 'rosidl_typesupport_fastrtps_c.*')),
)
