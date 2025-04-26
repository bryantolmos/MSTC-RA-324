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