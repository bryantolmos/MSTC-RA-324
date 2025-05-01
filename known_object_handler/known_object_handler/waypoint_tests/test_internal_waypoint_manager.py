#To verify internal methods such as publishing waypoints and storing them
#Will be converted to C++ once waypoint_manager_node.py is translated to C++

import pytest
from unittest.mock import MagicMock

from waypoint_manager_node import WaypointManagerNode
from geometry_msgs.msg import Pose, PoseArray

def test_waypoint_storage_and_publication():
    # Create the node
    node = WaypointManagerNode()

    # Create a mock publisher and replace the real one
    node.waypoints_pub_ = MagicMock()
    node.get_logger = MagicMock()  # suppress logs during test

    # Create a test pose and append it
    pose = Pose()
    pose.position.x = 1.0
    pose.position.y = 2.0
    pose.position.z = 3.0
    node.waypoints_.append(pose)

    # Call the method that should publish the list
    node.publish_current_waypoints()

    # Verify that publish was called exactly once
    assert node.waypoints_pub_.publish.call_count == 1

    # Extract the PoseArray that was published
    published_msg: PoseArray = node.waypoints_pub_.publish.call_args[0][0]

    # Verify the content of the published PoseArray
    assert isinstance(published_msg, PoseArray)
    assert len(published_msg.poses) == 1
    assert published_msg.poses[0].position.x == 1.0
    assert published_msg.poses[0].position.y == 2.0
    assert published_msg.poses[0].position.z == 3.0
