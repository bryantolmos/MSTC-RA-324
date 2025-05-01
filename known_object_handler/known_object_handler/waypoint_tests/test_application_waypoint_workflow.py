#Initial draft of Application Test
#Should be an end-to-end test to simulate the full workflow of waypoint_manager_node.py using ROS 2 test tools
#Will be rewritten in C++ once waypoint_manager_node.py is translated to C++

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PoseArray
from std_srvs.srv import Trigger
import pytest
import time

class TestWaypointApplication(Node):
    def __init__(self):
        super().__init__('test_waypoint_application')
        self.received_poses = None
        self.subscription = self.create_subscription(
            PoseArray,
            '/planned_waypoints',
            self.callback,
            10
        )

    def callback(self, msg):
        self.received_poses = msg

@pytest.mark.rostest
def test_end_to_end_workflow():
    rclpy.init()
    node = TestWaypointApplication()
    publisher = node.create_publisher(Point, '/web_selected_point_local', 10)

    # Wait for system to start
    time.sleep(2)

    # Publish a test point
    point = Point(x=1.0, y=0.5, z=0.2)
    publisher.publish(point)

    # Give time for processing
    time.sleep(2)

    # Check if PoseArray was received
    assert node.received_poses is not None
    assert len(node.received_poses.poses) > 0

    node.destroy_node()
    rclpy.shutdown()
