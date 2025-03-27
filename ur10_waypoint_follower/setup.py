from setuptools import setup, find_packages
import os

package_name = 'ur10_waypoint_follower'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    data_files=[
        (os.path.join('share', package_name, 'launch'),
         [os.path.join('launch', 'waypoint.launch.py')]),
        (os.path.join('share', package_name, 'resource'),
         [os.path.join('resource', 'waypoints.csv')]),
        (os.path.join('share', package_name), ['package.xml'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'coordinator_node = ur10_waypoint_follower.coordinator_node:main',
        ],
    },
)
