# test/test_object_snapshot_integration.py
#Integration testing
#"This launches the node, publishes a point cloud, calls the service, and verifies the output."

import rclpy
import unittest
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped
from object_snapshot_interfaces.srv import CaptureObject
import numpy as np
import time

from builtin_interfaces.msg import Time

class TestObjectSnapshotIntegration(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_integration_node')
        cls.client = cls.node.create_client(CaptureObject, 'capture_object')

        while not cls.client.wait_for_service(timeout_sec=5.0):
            cls.node.get_logger().info('Waiting for capture_object service...')

        cls.publisher = cls.node.create_publisher(PointCloud2, 'point_cloud', 10)
        time.sleep(1.0)  # wait for publishers/subscribers to connect

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def create_test_point_cloud(self):
        from sensor_msgs_py import point_cloud2
        from std_msgs.msg import Header

        points = [(0.5, 0.5, 0.5), (1.0, 1.0, 1.0), (2.0, 2.0, 2.0)]
        header = Header()
        header.frame_id = 'camera_frame'
        header.stamp = self.node.get_clock().now().to_msg()
        return point_cloud2.create_cloud_xyz32(header, points)

    def test_capture_service(self):
        pc_msg = self.create_test_point_cloud()

        # Publish point cloud
        self.publisher.publish(pc_msg)
        time.sleep(1.0)

        req = CaptureObject.Request()
        req.max_distance = 1.5

        future = self.client.call_async(req)

        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        self.assertTrue(future.done())
        res = future.result()

        self.assertTrue(res.success)
        self.assertAlmostEqual(res.pose.pose.position.x, 0.75, delta=0.1)
        self.assertEqual(res.pose.header.frame_id, 'base_link')

