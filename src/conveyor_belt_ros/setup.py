import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'conveyor_belt_ros'

data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    ('share/' + package_name + '/resource', glob('resource/*.yaml')),
    ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ('share/' + package_name + '/worlds', glob('worlds/*.wbt')),
    ('share/' + package_name + '/worlds', glob('worlds/*.proto')),
]

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='ROS2 package for conveyor belt simulation',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'conveyor_interface = conveyor_belt_ros.conveyor_interface:main',
        ],
    },
)
