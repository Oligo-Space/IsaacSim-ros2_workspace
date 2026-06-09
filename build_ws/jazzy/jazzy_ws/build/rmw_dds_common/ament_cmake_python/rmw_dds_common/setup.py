from setuptools import find_packages
from setuptools import setup

setup(
    name='rmw_dds_common',
    version='3.1.1',
    packages=find_packages(
        include=('rmw_dds_common', 'rmw_dds_common.*')),
)
