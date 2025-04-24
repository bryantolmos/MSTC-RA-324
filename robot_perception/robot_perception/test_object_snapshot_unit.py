# test/test_object_snapshot_unit.py
#For unit testing
#"This focuses on testing the logic inside the node (e.g., storing the snapshot, transforming it)."
#Need to make changes to script in areas that say "your_package_name"


import pytest
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from your_package_name.simple_object_snapshot_node import SimpleObjectSnapshotNode

def create_dummy_point_cloud():
    # Create a minimal dummy point cloud
    header = Header(frame_id='camera_frame')
    fields = [PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
              PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
              PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)]
    data = np.array([[1.0, 2.0, 3.0]], dtype=np.float32).tobytes()
    pc = PointCloud2(header=header, height=1, width=1, fields=fields,
                     is_bigendian=False, point_step=12, row_step=12,
                     data=data, is_dense=True)
    return pc

def test_snapshot_storage(monkeypatch):
    node = SimpleObjectSnapshotNode()

    # Mock transformation method if needed
    monkeypatch.setattr(node.tf_buffer, 'lookup_transform', lambda *args, **kwargs: None)

    # Simulate receiving a point cloud
    dummy_cloud = create_dummy_point_cloud()
    node.point_cloud_callback(dummy_cloud)

    assert node.latest_point_cloud is not None
    assert node.latest_point_cloud.header.frame_id == 'camera_frame'

    # Simulate service call
    request = type('DummyRequest', (), {})()  # empty request
    response = node.handle_snapshot_service(request, None)

    assert response.success
    assert b'\x00' not in response.snapshot.data  # Simple check that data is present
