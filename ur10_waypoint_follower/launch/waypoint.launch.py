from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ur10_waypoint_follower',
            executable='coordinator_node',
            name='coordinator',
            output='screen',
            parameters=[{
                'csv_path': 
                    os.path.join(
                        get_package_share_directory('ur10_waypoint_follower'),
                        'resource',
                        'waypoints.csv'
                    ),
                'offset': 0.05, 
                'use_sim_time': True 
            }]
        )
    ])
