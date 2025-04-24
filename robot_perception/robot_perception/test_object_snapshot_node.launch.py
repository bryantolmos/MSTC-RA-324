# test_object_snapshot_node.launch.py
#Will need to fill in strings with specific parameters

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='your_package_name',
            executable='simple_object_snapshot_node',
            name='simple_object_snapshot_node',
            parameters=[
                {'point_cloud_topic': '/zed/zed_node/point_cloud/cloud_registered'},
                {'world_frame': 'odom'},
                {'robot_base_frame': 'base_link'},
                {'filtered_cloud_topic': '/snapshot_filtered_cloud'},
                {'object_dims': [0.5, 0.3, 0.1]},
                {'min_filter_distance': 0.1},
                {'max_filter_distance': 0.6},
            ],
            output='screen'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_pub_camera_to_odom',
            arguments=['0', '0', '0.1', '0', '0', '0', 'camera_frame', 'odom']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_pub_odom_to_base',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link']
        )
    ])
