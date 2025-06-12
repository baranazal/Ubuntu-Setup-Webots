from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'agv_demo'

# Make sure the scripts directory is treated as a proper package
packages = [package_name]
# Add scripts if it exists and has __init__.py
if os.path.exists(os.path.join('scripts', '__init__.py')):
    packages.append('scripts')

setup(
    name=package_name,
    version='0.1.0',
    packages=packages,
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # Include world files
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.wbt')),
        # Include worlds proto files
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.proto')),
        # Include protos directory
        (os.path.join('share', package_name, 'protos'), glob('protos/*.proto')),
        # Include scripts
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.py')),
        # Include controllers
        (os.path.join('share', package_name, 'controllers', 'ned'), 
         glob('controllers/ned/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='AMR simulation using Webots and ROS2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'amr_controller = agv_demo.amr_controller:main',
            'belt_controller = agv_demo.belt_controller:main',
            'ned_controller = agv_demo.ned_controller:main',
        ],
    },
)