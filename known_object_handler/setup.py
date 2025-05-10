from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'known_object_handler'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'object_publisher_node = known_object_handler.object_publisher_node:main',
            'waypoint_manager_node = known_object_handler.waypoint_manager_node:main',
        ],
    },
)
