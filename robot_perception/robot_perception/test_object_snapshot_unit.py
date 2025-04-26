# test/test_object_snapshot_unit.py
#For unit testing
#"This focuses on testing the logic inside the node (e.g., storing the snapshot, transforming it)."

import unittest
import numpy as np
import open3d as o3d
from object_snapshot_node import ObjectSnapshotNode  # adjust import if needed
from geometry_msgs.msg import Point

class TestObjectSnapshotNodeUnit(unittest.TestCase):
    def setUp(self):
        self.node = ObjectSnapshotNode()

    def test_filter_point_cloud(self):
        # Create dummy point cloud
        points = np.array([
            [0.1, 0.2, 0.3],
            [1.0, 1.1, 1.2],
            [2.0, 2.1, 2.2],
            [3.0, 3.1, 3.2],
        ])
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)

        filtered = self.node.filter_point_cloud(pcd, max_distance=2.5)
        filtered_points = np.asarray(filtered.points)

        self.assertEqual(filtered_points.shape[0], 3)
        self.assertTrue((filtered_points == points[:3]).all())

    def test_calculate_centroid(self):
        points = np.array([
            [1.0, 1.0, 1.0],
            [2.0, 2.0, 2.0],
            [3.0, 3.0, 3.0],
        ])
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)

        centroid = self.node.calculate_centroid(pcd)

        expected = np.array([2.0, 2.0, 2.0])
        self.assertTrue(np.allclose([centroid.x, centroid.y, centroid.z], expected))

if __name__ == '__main__':
    unittest.main()

#Original Script in case it needs to be revisited

#V2
# import unittest
# from object_snapshot_node import ObjectSnapshotNode  # Adjust if this is wrapped in a class
# import rclpy

# class TestObjectSnapshotNodeUnit(unittest.TestCase):
#     @classmethod
#     def setUpClass(cls):
#         rclpy.init()

#     @classmethod
#     def tearDownClass(cls):
#         rclpy.shutdown()

#     def setUp(self):
#         self.node = ObjectSnapshotNode()

#     def tearDown(self):
#         self.node.destroy_node()

#     def test_initial_state(self):
#         self.assertIsNotNone(self.node)
#         self.assertTrue(hasattr(self.node, 'point_cloud_subscriber'))
#         self.assertTrue(hasattr(self.node, 'snapshot_service'))

#     def test_callback_processes_pointcloud(self):
#         # You can create a dummy PointCloud2 message here to test the callback
#         # e.g., self.node.point_cloud_callback(dummy_msg)
#         self.assertTrue(callable(self.node.point_cloud_callback))

# if __name__ == '__main__':
#     unittest.main()


#V1
#Need to make changes to below script in areas that say "your_package_name"

# import pytest
# import numpy as np
# from sensor_msgs.msg import PointCloud2, PointField
# from std_msgs.msg import Header
# from your_package_name.simple_object_snapshot_node import SimpleObjectSnapshotNode

# def create_dummy_point_cloud():
#     # Create a minimal dummy point cloud
#     header = Header(frame_id='camera_frame')
#     fields = [PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
#               PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
#               PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)]
#     data = np.array([[1.0, 2.0, 3.0]], dtype=np.float32).tobytes()
#     pc = PointCloud2(header=header, height=1, width=1, fields=fields,
#                      is_bigendian=False, point_step=12, row_step=12,
#                      data=data, is_dense=True)
#     return pc

# def test_snapshot_storage(monkeypatch):
#     node = SimpleObjectSnapshotNode()

#     # Mock transformation method if needed
#     monkeypatch.setattr(node.tf_buffer, 'lookup_transform', lambda *args, **kwargs: None)

#     # Simulate receiving a point cloud
#     dummy_cloud = create_dummy_point_cloud()
#     node.point_cloud_callback(dummy_cloud)

#     assert node.latest_point_cloud is not None
#     assert node.latest_point_cloud.header.frame_id == 'camera_frame'

#     # Simulate service call
#     request = type('DummyRequest', (), {})()  # empty request
#     response = node.handle_snapshot_service(request, None)

#     assert response.success
#     assert b'\x00' not in response.snapshot.data  # Simple check that data is present