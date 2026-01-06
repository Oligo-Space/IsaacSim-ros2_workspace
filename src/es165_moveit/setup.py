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
        # Find all files and put them in the share directory under the same folder name
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*')),
        (os.path.join('share', package_name, 'urdf'),
            glob('urdf/*')),
        (os.path.join('share', package_name, 'rviz'),
            glob('rviz/*')),
        
        #Finds all files in folder and subfolders
        (os.path.join('share', package_name, 'config'),
            [f for f in glob('config/**/*',recursive=True) if os.path.isfile(f)]),

        # Parse between visual and collision meshes as they have the same underlying names
        # Xacro file uses the visual meshes & collision_meshes shared folders to launch
        (os.path.join('share', package_name, 'visual_meshes'),
            [f for f in glob('meshes/visual/*',recursive=True) if os.path.isfile(f)]),
        (os.path.join('share', package_name, 'collision_meshes'),
            [f for f in glob('meshes/collision/*',recursive=True) if os.path.isfile(f)]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='oligo',
    maintainer_email='phillip@oligo.space',
    description='Arm Visualization and Interface with IsaacSim',
    license='',
    extras_require={
        'test': [
            'pytest',
        ],
    }, 
    entry_points={
        'console_scripts': [
            'joint_state_bridge = es165_moveit.joint_state_bridge:main',
            'zero_g_servo = es165_moveit.zero_g_servo:main',
            'torque_publisher = es165_moveit.torque_pub_test:main',
        ],
    },
)
