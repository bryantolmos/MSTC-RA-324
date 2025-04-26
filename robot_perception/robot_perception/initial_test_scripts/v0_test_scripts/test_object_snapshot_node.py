# test_object_snapshot_node.py

import rclpy
import pytest
from rclpy.node import Node
from std_srvs.srv import Trigger
from sensor_msgs.msg import PointCloud2
from builtin_interfaces.msg import Time
from std_msgs.msg import Header
from sensor_msgs_py import point_cloud2
import numpy as np
import time


@pytest.fixture(scope='module')
def rclpy_node():
    rclpy.init()
    node = rclpy.create_node('test_object_snapshot_node')
    yield node
    node.destroy_node()
    rclpy.shutdown()


def create_dummy_point_cloud(header_frame='camera_frame', num_points=10):
    header = Header()
    header.stamp = Time(sec=0, nanosec=0)
    header.frame_id = header_frame
    points = [(x * 0.05, x * 0.05, x * 0.05) for x in range(num_points)]
    return point_cloud2.create_cloud_xyz32(header, points)


def test_trigger_service_and_filtered_cloud(rclpy_node):
    # Publish dummy point cloud
    point_cloud_pub = rclpy_node.create_publisher(PointCloud2, '/zed/zed_node/point_cloud/cloud_registered', 10)
    time.sleep(1.0)  # Let the publisher initialize

    dummy_cloud = create_dummy_point_cloud()
    point_cloud_pub.publish(dummy_cloud)
    time.sleep(1.0)  # Let the node process it

    # Create service client
    client = rclpy_node.create_client(Trigger, '/capture_object_pose')
    assert client.wait_for_service(timeout_sec=5.0), "Service not available"

    request = Trigger.Request()
    future = client.call_async(request)

    rclpy.spin_until_future_complete(rclpy_node, future, timeout_sec=10.0)
    assert future.result() is not None, "Service call failed"
    assert future.result().success, f"Service response unsuccessful: {future.result().message}"
