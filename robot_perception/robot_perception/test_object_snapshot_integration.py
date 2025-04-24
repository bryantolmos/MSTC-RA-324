# test/test_object_snapshot_integration.py
#Integration testing
#"This launches the node, publishes a point cloud, calls the service, and verifies the output."


import rclpy
import pytest
from rclpy.node import Node
from std_srvs.srv import Trigger
from sensor_msgs.msg import PointCloud2
from launch_ros.actions import Node as LaunchNode
from launch import LaunchDescription
import numpy as np
import time

@pytest.fixture(scope='module')
def rclpy_node():
    rclpy.init()
    node = rclpy.create_node('test_node')
    yield node
    node.destroy_node()
    rclpy.shutdown()

def test_integration_snapshot_service(rclpy_node):
    # Wait for node to come up
    service_name = '/take_snapshot'
    end_time = time.time() + 5.0
    while time.time() < end_time:
        if rclpy_node.service_is_ready(service_name):
            break
        rclpy.spin_once(rclpy_node, timeout_sec=0.1)

    # Create service client
    client = rclpy_node.create_client(Trigger, service_name)
    assert client.wait_for_service(timeout_sec=5.0)

    # Publish a dummy point cloud
    pub = rclpy_node.create_publisher(PointCloud2, '/input/pointcloud', 10)
    dummy_pc = PointCloud2()
    dummy_pc.header.frame_id = 'camera_frame'
    dummy_pc.height = 1
    dummy_pc.width = 1
    dummy_pc.fields = []
    dummy_pc.is_dense = True
    dummy_pc.data = bytes(np.array([1, 2, 3, 4], dtype=np.uint8))
    pub.publish(dummy_pc)

    time.sleep(1.0)  # Give it time to be received

    # Call service
    req = Trigger.Request()
    future = client.call_async(req)
    rclpy.spin_until_future_complete(rclpy_node, future)
    response = future.result()
    assert response.success
