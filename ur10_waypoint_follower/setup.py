from setuptools import setup, find_packages
import os

package_name = 'ur10_waypoint_follower'

setup(
    name=package_name,
    version='0.0.1',
    py_modules=['test_scene_publisher'],
    packages=find_packages(),
    data_files=[
        (os.path.join('share', package_name, 'launch'),
         [os.path.join('launch', 'waypoint.launch.py')]),
        (os.path.join('share', package_name, 'resource'),
         [os.path.join('resource', 'waypoints.csv')]),
        # Include URDF files from the urdf subdirectory
        (os.path.join('share', package_name, 'urdf'),
         [
             os.path.join('urdf', 'ur.urdf.xacro'),
             os.path.join('urdf', 'welding_tool.urdf.xacro'),
         ]),
        # Include YAML configuration files from the config subdirectory
        (os.path.join('share', package_name, 'config'),
         [
             os.path.join('config', 'physical_parameters.yaml'),
             os.path.join('config', 'visual_parameters.yaml'),
         ]),
        (os.path.join('share', package_name), ['package.xml'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'coordinator_node = ur10_waypoint_follower.coordinator_node:main',
            'test_scene_publisher = test_scene_publisher:main', # If added to this package
        ],
    },
)
