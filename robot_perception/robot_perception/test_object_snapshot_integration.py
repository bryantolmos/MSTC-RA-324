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


#Original Scripts in case they needs to be revisited

#V2
# import rclpy
# import pytest
# from sensor_msgs.msg import PointCloud2
# from geometry_msgs.msg import TransformStamped
# from std_srvs.srv import Trigger

# def test_service_call_saves_snapshot(test_node, ros_test_node):
#     client = test_node.create_client(Trigger, '/object_snapshot/save_snapshot')
#     assert client.wait_for_service(timeout_sec=5.0)

#     req = Trigger.Request()
#     future = client.call_async(req)
#     rclpy.spin_until_future_complete(test_node, future)

#     assert future.result().success

# @pytest.fixture(scope='module')
# def test_node():
#     rclpy.init()
#     node = rclpy.create_node('test_object_snapshot_node')
#     yield node
#     node.destroy_node()
#     rclpy.shutdown()




#V1
# import rclpy
# import pytest
# from rclpy.node import Node
# from std_srvs.srv import Trigger
# from sensor_msgs.msg import PointCloud2
# from launch_ros.actions import Node as LaunchNode
# from launch import LaunchDescription
# import numpy as np
# import time

# @pytest.fixture(scope='module')
# def rclpy_node():
#     rclpy.init()
#     node = rclpy.create_node('test_node')
#     yield node
#     node.destroy_node()
#     rclpy.shutdown()

# def test_integration_snapshot_service(rclpy_node):
#     # Wait for node to come up
#     service_name = '/take_snapshot'
#     end_time = time.time() + 5.0
#     while time.time() < end_time:
#         if rclpy_node.service_is_ready(service_name):
#             break
#         rclpy.spin_once(rclpy_node, timeout_sec=0.1)

#     # Create service client
#     client = rclpy_node.create_client(Trigger, service_name)
#     assert client.wait_for_service(timeout_sec=5.0)

#     # Publish a dummy point cloud
#     pub = rclpy_node.create_publisher(PointCloud2, '/input/pointcloud', 10)
#     dummy_pc = PointCloud2()
#     dummy_pc.header.frame_id = 'camera_frame'
#     dummy_pc.height = 1
#     dummy_pc.width = 1
#     dummy_pc.fields = []
#     dummy_pc.is_dense = True
#     dummy_pc.data = bytes(np.array([1, 2, 3, 4], dtype=np.uint8))
#     pub.publish(dummy_pc)

#     time.sleep(1.0)  # Give it time to be received

#     # Call service
#     req = Trigger.Request()
#     future = client.call_async(req)
#     rclpy.spin_until_future_complete(rclpy_node, future)
#     response = future.result()
#     assert response.success
