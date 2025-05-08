import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException

from geometry_msgs.msg import Point, Pose, PoseStamped, PoseArray, Quaternion, TransformStamped
from std_srvs.srv import Trigger
from std_srvs.srv import Empty  # Import the Empty service type

# tf2 imports for handling transformations
import tf2_ros
from tf2_ros import TransformException, Buffer, TransformListener
import tf2_geometry_msgs # import tf2 helper for geometry message transformations
import math


def quaternion_from_vectors(v0, v1):
    """
    Returns a quaternion [x,y,z,w] that rotates vector v0 to v1.
    v0 and v1 should be 3-element iterables.
    """
    # normalize inputs
    def norm(u):
        return math.sqrt(sum(x*x for x in u))
    v0n = [x / norm(v0) for x in v0]
    v1n = [x / norm(v1) for x in v1]

    # axis = cross(v0, v1)
    axis = [
        v0n[1]*v1n[2] - v0n[2]*v1n[1],
        v0n[2]*v1n[0] - v0n[0]*v1n[2],
        v0n[0]*v1n[1] - v0n[1]*v1n[0],
    ]
    axis_len = norm(axis)

    # handle nearly parallel or anti-parallel vectors
    if axis_len < 1e-6:
        # if they’re anti-parallel, rotate 180° around any perpendicular axis
        dot = sum(a*b for a,b in zip(v0n, v1n))
        if dot < 0:
            # pick an arbitrary orthogonal axis:
            axis = [1.0, 0.0, 0.0] if abs(v0n[0]) < 0.9 else [0.0, 1.0, 0.0]
            # re-cross to ensure perpendicular
            axis = [
                v0n[1]*axis[2] - v0n[2]*axis[1],
                v0n[2]*axis[0] - v0n[0]*axis[2],
                v0n[0]*axis[1] - v0n[1]*axis[0]
            ]
            axis_len = norm(axis)
        else:
            # vectors are the same—no rotation needed
            return [0.0, 0.0, 0.0, 1.0]

    axis = [x / axis_len for x in axis]
    # angle between v0 and v1
    dot = sum(a*b for a,b in zip(v0n, v1n))
    # numerical safety
    dot = max(min(dot, 1.0), -1.0)
    angle = math.acos(dot)

    sin_half = math.sin(angle / 2.0)
    cos_half = math.cos(angle / 2.0)
    return [
        axis[0] * sin_half,
        axis[1] * sin_half,
        axis[2] * sin_half,
        cos_half
    ]


class WaypointManagerNode(Node):
    """
    Listens for local points clicked relative to a known object frame,
    transforms them into Pose waypoints in the robot base frame,
    and makes the orientation of the waypoints face the object.
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
            # step 1: look up the necessary transforms
            transform_timeout = Duration(seconds=1.0)
            transform_object_to_base = self.tf_buffer_.lookup_transform(
                 self.robot_base_frame_, # target frame
                 self.object_frame_,     # source frame
                 rclpy.time.Time(),      # use time 0 to get the latest available transform
                 timeout=transform_timeout
            )

            # step 2: create a posestamped message for the input point (in object frame)
            pose_in_object_frame = PoseStamped()
            pose_in_object_frame.header.stamp = self.get_clock().now().to_msg() # stamp with current time
            pose_in_object_frame.header.frame_id = self.object_frame_
            pose_in_object_frame.pose.position = msg # use the received point for the position
            # set a default orientation
            pose_in_object_frame.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

            # step 3: transform the pose to the robot base frame
            waypoint_pose_stamped_in_base = tf2_geometry_msgs.do_transform_pose_stamped(
                pose_in_object_frame, # the posestamped object to transform
                transform_object_to_base # the transform to apply
            )

            # step 4: calculate orientation to face the object
            # Get the waypoint's position in the base frame
            waypoint_position_base = waypoint_pose_stamped_in_base.pose.position

            # Get the object's position in the base frame
            try:
                object_transform = self.tf_buffer_.lookup_transform(
                    self.robot_base_frame_,
                    self.object_frame_,
                    rclpy.time.Time(),
                    timeout=transform_timeout
                )
                object_position_base = object_transform.transform.translation
            except TransformException as ex:
                self.get_logger().warn(f"Could not get object transform: {ex}")
                return

            # Calculate the vector from the waypoint to the object
            vector_to_object = [
                object_position_base.x - waypoint_position_base.x,
                object_position_base.y - waypoint_position_base.y,
                object_position_base.z - waypoint_position_base.z
            ]

            # Normalize the vector
            norm = math.sqrt(sum(x*x for x in vector_to_object))
            if norm > 1e-6:  # Avoid division by zero
                vector_to_object_normalized = [x / norm for x in vector_to_object]
            else:
                vector_to_object_normalized = [0.0, 0.0, 1.0] # Default direction

            # Define a default "up" direction (e.g., world's Z-axis in base frame)
            up_direction = [0.0, 0.0, 1.0]

            # Calculate the rotation to align the end-effector's Z-axis with the vector to the object
            rotation = quaternion_from_vectors([0.0, 0.0, 1.0], vector_to_object_normalized)            
            waypoint_pose_stamped_in_base.pose.orientation.x = rotation[0]
            waypoint_pose_stamped_in_base.pose.orientation.y = rotation[1]
            waypoint_pose_stamped_in_base.pose.orientation.z = rotation[2]
            waypoint_pose_stamped_in_base.pose.orientation.w = rotation[3]

            self.get_logger().info(f"Transformed waypoint in {self.robot_base_frame_} (facing object): "
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