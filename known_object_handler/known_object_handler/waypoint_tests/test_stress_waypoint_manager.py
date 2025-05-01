#Initial Stress Test script draft
#Simulates rapidly publishing a large number of points
#Tests for buffer, transform, and memory handling

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import time

class StressTestPublisher(Node):
    def __init__(self):
        super().__init__('stress_test_publisher')
        self.publisher_ = self.create_publisher(Point, '/web_selected_point_local', 10)

    def send_points(self, count=100):
        for i in range(count):
            msg = Point(x=float(i), y=float(i)/2.0, z=0.1)
            self.publisher_.publish(msg)
            time.sleep(0.01)  # Simulate fast, but not unrealistic publishing rate

def test_stress_waypoint_handling():
    rclpy.init()
    node = StressTestPublisher()
    time.sleep(2)  # To give the system time to boot
    node.send_points(count=200)  # Number of sent points can be adjusted as needed
    node.get_logger().info("Stress test completed.")
    node.destroy_node()
    rclpy.shutdown()
