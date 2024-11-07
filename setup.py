from setuptools import setup

package_name = 'robot_urdf'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Fabio',
    maintainer_email='hidden',
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
