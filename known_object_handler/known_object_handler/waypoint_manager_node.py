import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException

from geometry_msgs.msg import Point, Pose, PoseStamped, PoseArray, Quaternion, TransformStamped
from std_srvs.srv import Trigger

# tf2 imports for handling transformations
import tf2_ros
from tf2_ros import TransformException, Buffer, TransformListener
import tf2_geometry_msgs # import tf2 helper for geometry message transformations

class WaypointManagerNode(Node):
    """
    Listens for local points clicked relative to a known object frame,
    transforms them into Pose waypoints in the robot base frame,
    and makes the list available for planning.
    """
    def __init__(self):
        super().__init__('waypoint_manager_node')

        # declare node parameters
        self.declare_parameter('robot_base_frame', 'base_link')
        self.declare_parameter('known_object_frame', 'known_object_frame')
        self.declare_parameter('local_point_topic', '/web_selected_point_local')
        self.declare_parameter('waypoint_topic', '/planned_waypoints') # topic for publishing the final waypoints

        # retrieve parameter values
        self.robot_base_frame_ = self.get_parameter('robot_base_frame').get_parameter_value().string_value
        self.object_frame_ = self.get_parameter('known_object_frame').get_parameter_value().string_value
        local_point_topic = self.get_parameter('local_point_topic').get_parameter_value().string_value
        waypoint_topic = self.get_parameter('waypoint_topic').get_parameter_value().string_value

        # log parameters for verification
        self.get_logger().info("--- Waypoint Manager Node Parameters ---")
        self.get_logger().info(f"Robot Base Frame: {self.robot_base_frame_}")
        self.get_logger().info(f"Object Frame: {self.object_frame_}")
        self.get_logger().info(f"Listening for local points on: {local_point_topic}")
        self.get_logger().info(f"Publishing waypoints (PoseArray) to: {waypoint_topic}")
        self.get_logger().info("-------------------------------------")

        # internal storage for waypoints
        self.waypoints_ = [] # list to store waypoint poses (geometry_msgs/Pose)

        # setup tf2 buffer and listener
        self.tf_buffer_ = Buffer()
        self.tf_listener_ = TransformListener(self.tf_buffer_, self)

        # setup publishers
        # publisher for the waypoint list (uses latching qos)
        latching_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.waypoints_pub_ = self.create_publisher(PoseArray, waypoint_topic, latching_qos)

        # setup subscribers
        self.point_sub_ = self.create_subscription(
            Point,
            local_point_topic,
            self.local_point_callback,
            10 # standard qos depth is usually sufficient for ui interactions
        )

        # setup service servers
        self.clear_service_ = self.create_service(
            Trigger,
            '/clear_waypoints',
            self.clear_waypoints_callback
        )
        # note: publishing now occurs automatically on update or clear, no dedicated service

        self.get_logger().info("Waypoint Manager Node started.")
        self.publish_current_waypoints() # publish an empty list on startup

    def local_point_callback(self, msg: Point):
        """Callback for receiving a point relative to the object frame."""
        self.get_logger().info(f"Received local point: [{msg.x:.3f}, {msg.y:.3f}, {msg.z:.3f}]")

        try:
            # step 1: look up the necessary transform
            # get the transform from the object frame to the robot base frame
            transform_timeout = Duration(seconds=1.0)
            transform_object_to_base = self.tf_buffer_.lookup_transform(
                 self.robot_base_frame_, # target frame
                 self.object_frame_,     # source frame
                 rclpy.time.Time(),      # use time 0 to get the latest available transform
                 timeout=transform_timeout
            )

            # step 2: create a posestamped message for the input point
            pose_in_object_frame = PoseStamped()
            pose_in_object_frame.header.stamp = self.get_clock().now().to_msg() # stamp with current time
            pose_in_object_frame.header.frame_id = self.object_frame_
            pose_in_object_frame.pose.position = msg # use the received point for the position
            # set a default orientation (aligned with object frame)
            pose_in_object_frame.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

            # step 3: transform the pose to the robot base frame
            waypoint_pose_stamped_in_base = tf2_geometry_msgs.do_transform_pose_stamped(
                pose_in_object_frame, # the posestamped object to transform
                transform_object_to_base # the transform to apply
            )

            # step 4: orientation handling (currently uses transformed orientation)
            # the transform includes the object's orientation relative to the base.
            # current orientation is correct relative to the object frame's orientation.
            # future enhancement: calculate orientation to point towards object if needed (e.g., using atan2).

            self.get_logger().info(f"Transformed waypoint in {self.robot_base_frame_}: "
                                   f"P[{waypoint_pose_stamped_in_base.pose.position.x:.3f}, "
                                   f"{waypoint_pose_stamped_in_base.pose.position.y:.3f}, "
                                   f"{waypoint_pose_stamped_in_base.pose.position.z:.3f}] "
                                   f"O[{waypoint_pose_stamped_in_base.pose.orientation.x:.3f}, "
                                   f"{waypoint_pose_stamped_in_base.pose.orientation.y:.3f}, "
                                   f"{waypoint_pose_stamped_in_base.pose.orientation.z:.3f}, "
                                   f"{waypoint_pose_stamped_in_base.pose.orientation.w:.3f}]")


            # step 5: store the resulting pose (not posestamped) in the list
            self.waypoints_.append(waypoint_pose_stamped_in_base.pose)

            # step 6: publish the updated waypoint list
            self.publish_current_waypoints()

        except TransformException as ex:
            # log error if transform lookup fails
            self.get_logger().error(f"Could not transform point from {self.object_frame_} "
                                   f"to {self.robot_base_frame_}: {ex}")
        except Exception as e:
             # log any other unexpected errors
             self.get_logger().error(f"Unexpected error in local_point_callback: {e}", exc_info=True)


    def clear_waypoints_callback(self, request, response):
        """Clears the stored waypoints."""
        self.get_logger().info("Clearing waypoints.")
        self.waypoints_ = []
        self.publish_current_waypoints() # publish the now-empty list
        response.success = True
        response.message = "Waypoints cleared."
        return response

    def publish_current_waypoints(self):
        """Publishes the current list of waypoints as a PoseArray."""
        pose_array_msg = PoseArray()
        pose_array_msg.header.stamp = self.get_clock().now().to_msg()
        # ensure header frame matches the frame of the poses
        pose_array_msg.header.frame_id = self.robot_base_frame_
        # assign the internal list to the message field
        pose_array_msg.poses = self.waypoints_

        self.waypoints_pub_.publish(pose_array_msg)
        self.get_logger().info(f"Published {len(self.waypoints_)} waypoints to {self.waypoints_pub_.topic_name}")


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = WaypointManagerNode()
        # keep the node running until interrupted
        rclpy.spin(node)
    except KeyboardInterrupt:
        # handle ctrl+c cleanly
        pass
    except ExternalShutdownException:
        # handle external shutdown requests
        pass
    finally:
        # ensure node is destroyed and rclpy is shut down on exit
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

# standard python entry point
if __name__ == '__main__':
    main()