import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
import random
import time
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class DummyPointCloudPublisher(Node):
    def __init__(self):
        super().__init__('dummy_pointcloud_publisher')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.publisher_ = self.create_publisher(PointCloud2, '/dummy_pointcloud', qos_profile)
        self.object_pose_publisher_ = self.create_publisher(PoseStamped, '/detected_object_pose', qos_profile)
        self.timer = self.create_timer(1.0, self.publish_dummy_data) # Publish every 1 second

    def publish_dummy_data(self):
        # Generate dummy point cloud data
        num_points = 100
        points = []
        for i in range(num_points):
            x = random.uniform(0.5, 1.5)  # Example range in front of the robot
            y = random.uniform(-0.5, 0.5)
            z = random.uniform(0.1, 0.5)
            points.append([x, y, z])

        # Create PointCloud2 message
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        header_msg = rclpy.time.Time().to_msg()
        header_msg.frame_id = 'camera_link' # Adjust to your robot's camera frame
        pointcloud_msg = point_cloud2.create_cloud(header_msg, fields, points)
        self.publisher_.publish(pointcloud_msg)
        self.get_logger().info(f'Published dummy PointCloud2 with {num_points} points')

        # Generate a dummy object pose (you'll extract this from the point cloud later)
        object_pose_msg = PoseStamped()
        object_pose_msg.header.stamp = self.get_clock().now().to_msg() # Correct way to set timestamp
        object_pose_msg.header.frame_id = 'base_link' # Assuming object pose is in base_link frame
        object_pose_msg.pose.position.x = random.uniform(0.7, 1.3)
        object_pose_msg.pose.position.y = random.uniform(-0.3, 0.3)
        object_pose_msg.pose.position.z = random.uniform(0.2, 0.4)
        object_pose_msg.pose.orientation.w = 1.0 # No specific orientation for now
        self.object_pose_publisher_.publish(object_pose_msg)
        self.get_logger().info(f'Published dummy object pose: {object_pose_msg.pose.position}')


def main(args=None):
    rclpy.init(args=args)
    node = DummyPointCloudPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()