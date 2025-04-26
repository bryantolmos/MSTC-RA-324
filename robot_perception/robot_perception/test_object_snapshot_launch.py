# test/test_object_snapshot_launch.py
#Specifically designed to support integration test; starts SimpleObjectSnapshotNode + any needed dependencies


import os
import pytest
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_testing_ros.tools import basic_launch_tests

@pytest.mark.rostest
def generate_test_description():
    return LaunchDescription([
        Node(
            package='object_snapshot',
            executable='object_snapshot_node',
            name='object_snapshot_node',
            output='screen',
        ),
    ]), {}

# Include generic checks for life, services, etc.
# For advanced launch checks, expand with pytest fixtures



#Original Script in case it needs to be revisited
#Changes to be made in regards to variable parameters if needed (such as areas with "your_package_name")

# from launch import LaunchDescription
# from launch_ros.actions import Node
# from launch.actions import DeclareLaunchArgument
# from launch.substitutions import LaunchConfiguration

# def generate_launch_description():
#     return LaunchDescription([
#         # Optional: Declare arguments for flexibility
#         DeclareLaunchArgument('use_sim_time', default_value='false'),
        
#         # Node under test
#         Node(
#             package='your_package_name',
#             executable='simple_object_snapshot_node',
#             name='simple_object_snapshot_node',
#             output='screen',
#             parameters=[{
#                 'use_sim_time': LaunchConfiguration('use_sim_time')
#             }]
#         ),

#         # Static transform publisher (e.g., from camera_frame to world)
#         Node(
#             package='tf2_ros',
#             executable='static_transform_publisher',
#             name='static_tf_pub',
#             arguments=['0', '0', '0', '0', '0', '0', 'camera_frame', 'world']
#         )
#     ])
