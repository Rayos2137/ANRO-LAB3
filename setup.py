import os
from glob import glob
from setuptools import setup

package_name = 'my_robot_rviz'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),  # <--- WAŻNE!
        ('share/' + package_name + '/urdf', glob('urdf/*.urdf')),
        ('share/' + package_name + '/urdf', glob('urdf/*.xacro'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='PePe',
    maintainer_email='your_email@example.com',
    description='My first RViz2 project',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': 
        ['forward_kinematics = my_robot_rviz.forward_kinematics:main'],
    },
)
