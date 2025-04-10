import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose
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
        self.timer = self.create_timer(10.0, self.publish_sequence) # Publish sequence every 10 seconds
        self.sequence_index = 0
        self.start_pose = Pose()
        self.start_pose.position.x = 0.6  # Example starting position
        self.start_pose.position.y = 0.0
        self.start_pose.position.z = 0.6
        # Example starting orientation (no rotation)
        self.start_pose.orientation.x = 0.0
        self.start_pose.orientation.y = 0.0
        self.start_pose.orientation.z = 0.0
        self.start_pose.orientation.w = 0.0

    def publish_sequence(self):
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'base_link'

        target_pose = Pose()

        if self.sequence_index == 0:
            target_pose = self.start_pose
            self.get_logger().info(f'Publishing: Start Position\n')
        elif self.sequence_index == 1:
            target_pose = Pose()
            target_pose.position.x = self.start_pose.position.x - 0.2  # Move forward (negative X)
            target_pose.position.y = self.start_pose.position.y + 0.1  # Move slightly to the left (positive Y)
            target_pose.position.z = self.start_pose.position.z - 0.1  # Move down
            target_pose.orientation = self.start_pose.orientation
            self.get_logger().info(f'Publishing: Move 1\n')
        elif self.sequence_index == 2:
            target_pose = Pose()
            target_pose.position.x = self.start_pose.position.x - 0.2  # Keep the same X
            target_pose.position.y = self.start_pose.position.y - 0.3  # Move further to the right (negative Y)
            target_pose.position.z = self.start_pose.position.z - 0.1  # Keep the same Z
            target_pose.orientation = self.start_pose.orientation
            self.get_logger().info(f'Publishing: Move 2 (Sideways)\n')
        elif self.sequence_index == 3:
            target_pose = Pose()
            target_pose.position.x = self.start_pose.position.x + 0.3  # Move backward (positive X)
            target_pose.position.y = self.start_pose.position.y - 0.3  # Keep the same Y
            target_pose.position.z = self.start_pose.position.z + 0.2  # Move up
            target_pose.orientation = self.start_pose.orientation
            self.get_logger().info(f'Publishing: Move 3 (Up and Back)\n')
        elif self.sequence_index == 4:
            target_pose = self.start_pose # Return to start
            target_pose.orientation = self.start_pose.orientation
            self.get_logger().info(f'Publishing: Return to Start\n')

        pose_msg.pose = target_pose
        self.publisher_.publish(pose_msg)

        self.sequence_index += 1
        if self.sequence_index > 4:
            self.sequence_index = 0 # Loop the sequence

def main(args=None):
    rclpy.init(args=args)
    node = DummyPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()