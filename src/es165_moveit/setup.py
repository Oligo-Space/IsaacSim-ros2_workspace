from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'es165_moveit'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        (os.path.join('share', package_name, 'urdf'),
            glob('urdf/*')),
        (os.path.join('share', package_name, 'meshes'),
            [f for f in glob('meshes/**/*',recursive=True) if os.path.isfile(f)]),
        (os.path.join('share', package_name, 'config'),
            [f for f in glob('config/**/*',recursive=True) if os.path.isfile(f)]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='oligo',
    maintainer_email='kguo@oligo.space',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
