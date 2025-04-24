# test/test_object_snapshot_launch.py
#Specifically designed to support integration test; starts SimpleObjectSnapshotNode + any needed dependencies
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

import rclpy
import pytest
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import TransformStamped
from std_srvs.srv import Trigger

def test_service_call_saves_snapshot(test_node, ros_test_node):
    client = test_node.create_client(Trigger, '/object_snapshot/save_snapshot')
    assert client.wait_for_service(timeout_sec=5.0)

    req = Trigger.Request()
    future = client.call_async(req)
    rclpy.spin_until_future_complete(test_node, future)

    assert future.result().success

@pytest.fixture(scope='module')
def test_node():
    rclpy.init()
    node = rclpy.create_node('test_object_snapshot_node')
    yield node
    node.destroy_node()
    rclpy.shutdown()
