from setuptools import setup

package_name = 'dummy_pointcloud_publisher'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='Publishes dummy 3D point cloud data for MoveIt testing.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dummy_pointcloud_publisher = dummy_pointcloud_publisher.dummy_publisher:main',
        ],
    },
)