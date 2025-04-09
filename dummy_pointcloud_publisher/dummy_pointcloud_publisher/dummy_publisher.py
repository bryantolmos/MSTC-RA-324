import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import random
import time
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class DummyPosePublisher(Node):
    def __init__(self):
        super().__init__('dummy_pose_publisher')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.publisher_ = self.create_publisher(PoseStamped, '/dummy_target_pose', qos_profile)
        self.timer = self.create_timer(5.0, self.publish_dummy_pose) # Publish every 1 second

    def publish_dummy_pose(self):
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'base_link' # Adjust to your desired frame

        # Generate random dummy coordinates and orientation
        pose_msg.pose.position.x = random.uniform(0.5, 1.0)
        pose_msg.pose.position.y = random.uniform(-0.3, 0.3)
        pose_msg.pose.position.z = random.uniform(0.2, 0.6)
        pose_msg.pose.orientation.x = random.uniform(-1.0, 1.0)
        pose_msg.pose.orientation.y = random.uniform(-1.0, 1.0)
        pose_msg.pose.orientation.z = random.uniform(-1.0, 1.0)
        pose_msg.pose.orientation.w = random.uniform(-1.0, 1.0)

        # Normalize the quaternion
        norm = (pose_msg.pose.orientation.x**2 +
                pose_msg.pose.orientation.y**2 +
                pose_msg.pose.orientation.z**2 +
                pose_msg.pose.orientation.w**2)**0.5
        pose_msg.pose.orientation.x /= norm
        pose_msg.pose.orientation.y /= norm
        pose_msg.pose.orientation.z /= norm
        pose_msg.pose.orientation.w /= norm

        self.publisher_.publish(pose_msg)
        self.get_logger().info(f'Published dummy pose: {pose_msg.pose.position}, {pose_msg.pose.orientation}\n')

def main(args=None):
    rclpy.init(args=args)
    node = DummyPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()