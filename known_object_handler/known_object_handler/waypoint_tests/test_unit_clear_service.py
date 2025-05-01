#To test a complete function in isolation. For this file, it will be for clear_waypoints_callback


import rclpy
from waypoint_manager_node import WaypointManagerNode
from std_srvs.srv import Trigger

class DummyRequest:
    pass

def test_clear_waypoints_callback():
    rclpy.init()
    node = WaypointManagerNode()

    # Simulate existing waypoints
    from geometry_msgs.msg import Pose
    pose = Pose()
    node.waypoints_.append(pose)
    assert len(node.waypoints_) == 1

    request = DummyRequest()
    response = Trigger.Response()
    response = node.clear_waypoints_callback(request, response)

    assert response.success is True
    assert response.message == "Waypoints cleared."
    assert len(node.waypoints_) == 0

    node.destroy_node()
    rclpy.shutdown()
