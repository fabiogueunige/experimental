from setuptools import find_packages, setup
from setuptools import setup
from glob       import glob
import os

package_name = 'robot_urdf'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fabio',
    maintainer_email='s5004782@studenti.unige.it',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_rotation_node = robot_urdf.robot_rotation_node:main',
            'marker_sub_node = robot_urdf.marker_sub_node:main'
        ],
    },
)
