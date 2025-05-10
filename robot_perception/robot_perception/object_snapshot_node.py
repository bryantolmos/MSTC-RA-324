import rclpy
from rclpy.node import Node
# import qos settings for publishers/subscribers
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
# import duration for timeouts and lifetimes
from rclpy.duration import Duration
# import time utilities
import rclpy.time
# import exception for clean shutdown
from rclpy.executors import ExternalShutdownException

# import numpy for numerical operations, especially on point clouds
import numpy as np
# import math for basic math functions if needed
import math

# import standard ros message and service types
from sensor_msgs.msg import PointCloud2
from std_srvs.srv import Trigger # standard trigger service for simple activation
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion, Vector3, TransformStamped
from visualization_msgs.msg import Marker # for publishing visual markers in rviz
from moveit_msgs.msg import CollisionObject # for publishing collision objects for planning scenes
from shape_msgs.msg import SolidPrimitive # defines shapes for markers and collision objects
from std_msgs.msg import Header # standard header for timestamp and frame_id
from std_msgs.msg import ColorRGBA # standard color message

# import helper library for working with PointCloud2 data
from sensor_msgs_py import point_cloud2 as pc2

# import tf2 libraries for handling coordinate transformations
import tf2_ros
from tf2_ros import TransformException, Buffer, TransformListener
# import helper for transforming geometry_msgs with tf2
import tf2_geometry_msgs

class ObjectSnapshotNode(Node):
    """
    captures a snapshot of nearby points upon a trigger service call.
    calculates the centroid of these points.
    publishes a visualization marker, a moveit collision object,
    the calculated pose relative to the robot's base frame,
    and the filtered pointcloud2 data used for the calculation.
    """
    def __init__(self):
        # initialize the node with a name
        super().__init__('simple_object_snapshot_node')

        # --- parameter declaration ---
        # declare parameters with default values; these can be overridden at launch
        self.declare_parameter('point_cloud_topic', '/zed/zed_node/point_cloud/cloud_registered') # input topic for point cloud data
        self.declare_parameter('world_frame', 'odom') # reference frame for global poses (e.g., 'odom', 'map')
        self.declare_parameter('robot_base_frame', 'base_link') # reference frame attached to the robot's base
        self.declare_parameter('object_dims', [0.5, 0.3, 0.1]) # estimated dimensions [l, h, w] for marker/collision object
        self.declare_parameter('max_filter_distance', 0.5) # maximum distance from sensor origin to consider points
        self.declare_parameter('min_filter_distance', 0.1) # minimum distance from sensor origin to consider points
        self.declare_parameter('collision_padding', 0.02) # padding added to object dimensions for collision safety
        self.declare_parameter('marker_color_r', 0.1) # red component of marker color (0.0-1.0)
        self.declare_parameter('marker_color_g', 0.7) # green component of marker color (0.0-1.0)
        self.declare_parameter('marker_color_b', 0.1) # blue component of marker color (0.0-1.0)
        self.declare_parameter('marker_color_a', 0.7) # alpha component (transparency) of marker color (0.0-1.0)
        self.declare_parameter('filtered_cloud_topic', '/snapshot_filtered_cloud') # topic to publish the filtered point cloud on

        # --- retrieve parameter values ---
        # get the actual values of the parameters after potential overrides
        self.point_cloud_topic_ = self.get_parameter('point_cloud_topic').get_parameter_value().string_value
        self.world_frame_ = self.get_parameter('world_frame').get_parameter_value().string_value
        self.robot_base_frame_ = self.get_parameter('robot_base_frame').get_parameter_value().string_value
        self.object_dims_ = self.get_parameter('object_dims').get_parameter_value().double_array_value
        # store squared distances for efficiency in filtering
        self.max_dist_sq_ = self.get_parameter('max_filter_distance').get_parameter_value().double_value ** 2
        self.min_dist_sq_ = self.get_parameter('min_filter_distance').get_parameter_value().double_value ** 2
        self.collision_padding_ = self.get_parameter('collision_padding').get_parameter_value().double_value
        # construct the colorrgba message from individual parameters
        self.marker_color_ = ColorRGBA(
            r=self.get_parameter('marker_color_r').get_parameter_value().double_value,
            g=self.get_parameter('marker_color_g').get_parameter_value().double_value,
            b=self.get_parameter('marker_color_b').get_parameter_value().double_value,
            a=self.get_parameter('marker_color_a').get_parameter_value().double_value
        )
        self.filtered_cloud_topic_ = self.get_parameter('filtered_cloud_topic').get_parameter_value().string_value

        # log the initialized parameters for confirmation
        self.get_logger().info("--- simple object snapshot node parameters initialized ---")
        self.get_logger().info(f"input point cloud topic: {self.point_cloud_topic_}")
        self.get_logger().info(f"output filtered cloud topic: {self.filtered_cloud_topic_}") # log the new topic name
        self.get_logger().info(f"world frame: {self.world_frame_}")
        self.get_logger().info(f"robot base frame: {self.robot_base_frame_}")
        # log other parameters as needed for debugging

        # --- tf2 setup ---
        # create a tf2 buffer to store transforms
        self.tf_buffer_ = Buffer()
        # create a tf2 listener to receive transforms
        self.tf_listener_ = TransformListener(self.tf_buffer_, self)

        # --- publishers ---
        # create publishers for various outputs
        self.marker_pub_ = self.create_publisher(Marker, '/detected_object_marker', 10) # marker for rviz
        self.collision_pub_ = self.create_publisher(CollisionObject, '/collision_object', 10) # collision object for moveit
        self.object_pose_pub_ = self.create_publisher(PoseStamped, '/object_pose_in_base_link', 10) # pose relative to robot base
        # publisher for the filtered point cloud used in calculation
        self.filtered_cloud_pub_ = self.create_publisher(PointCloud2, self.filtered_cloud_topic_, 10)

        # --- service server ---
        # create a service server to trigger the capture process
        self.trigger_service_ = self.create_service(
            Trigger, # using the standard trigger service type
            '/capture_object_pose', # service name to call
            self.capture_callback) # function to execute when called

        self.get_logger().info(f"simple object snapshot node started. ready on '/capture_object_pose'. publishing filtered cloud to '{self.filtered_cloud_topic_}'.")

    # --- capture callback ---
    # this function is executed when the '/capture_object_pose' service is called
    def capture_callback(self, request, response):
        self.get_logger().info("capture request received.")
        try:
            # 1. get point cloud
            # wait for a single message on the specified point cloud topic
            self.get_logger().info(f"waiting for point cloud message on {self.point_cloud_topic_}...")
            point_cloud_msg = self.wait_for_message(PointCloud2, self.point_cloud_topic_, timeout_sec=5.0)
            # check if a message was actually received
            if point_cloud_msg is None:
                raise TimeoutError(f"timeout waiting for message on {self.point_cloud_topic_}")
            # record the time the cloud was processed (approximately)
            now = self.get_clock().now()
            # get the frame id from the received message header
            source_frame = point_cloud_msg.header.frame_id
            self.get_logger().info(f"point cloud received. frame: {source_frame}, points: {point_cloud_msg.width * point_cloud_msg.height}")

            # 2. filter points based on distance
            self.get_logger().info("filtering points based on distance...")
            filtered_points = [] # list to store valid [x, y, z] points
            # use pc2 helper to iterate through points efficiently
            points_generator = pc2.read_points(point_cloud_msg, field_names=("x", "y", "z"), skip_nans=True)
            for p in points_generator:
                # calculate squared distance (faster than sqrt)
                dist_sq = p[0]**2 + p[1]**2 + p[2]**2
                # check if the point is within the min/max distance thresholds
                if self.min_dist_sq_ <= dist_sq <= self.max_dist_sq_:
                    filtered_points.append([p[0], p[1], p[2]]) # store as list [x, y, z]

            # check if any points remained after filtering
            if not filtered_points:
                self.get_logger().warn("no points found within the specified filter distance range.")
                response.success = False
                response.message = "no points found within filter distance."
                # publish an empty point cloud message to indicate no points were found
                empty_header = Header(stamp=now.to_msg(), frame_id=source_frame)
                empty_cloud_msg = pc2.create_cloud_xyz32(empty_header, []) # create cloud with empty point list
                self.filtered_cloud_pub_.publish(empty_cloud_msg)
                self.get_logger().info("published empty filtered cloud.")
                return response # exit the callback
            self.get_logger().info(f"filtering complete: {len(filtered_points)} points remaining.")

            # --- publish filtered point cloud ---
            # publish the point cloud containing only the points used for calculation
            self.get_logger().info("publishing the filtered point cloud...")
            # create a header using the current time and the original cloud's frame id
            filtered_header = Header(stamp=now.to_msg(), frame_id=source_frame)
            # create the PointCloud2 message using the helper function
            filtered_cloud_msg = pc2.create_cloud_xyz32(filtered_header, filtered_points)
            # publish the message
            self.filtered_cloud_pub_.publish(filtered_cloud_msg)
            self.get_logger().info(f"filtered point cloud published to {self.filtered_cloud_topic_}.")
            # ---------------------------------------------

            # 3. calculate centroid pose in the original camera frame
            self.get_logger().info("calculating centroid pose in camera frame...")
            # convert list of points to a numpy array for efficient calculation
            points_np = np.array(filtered_points)
            # calculate the mean along the columns (x, y, z)
            centroid = np.mean(points_np, axis=0)
            # create a PoseStamped message for the centroid
            centroid_pose_camera = PoseStamped()
            # use time(0) for the header stamp when the transform is intended for the latest available transform
            centroid_pose_camera.header.stamp = rclpy.time.Time(seconds=0, nanoseconds=0).to_msg()
            centroid_pose_camera.header.frame_id = source_frame # frame is the original point cloud frame
            # set the position to the calculated centroid coordinates
            centroid_pose_camera.pose.position = Point(x=float(centroid[0]), y=float(centroid[1]), z=float(centroid[2]))
            # set a default orientation (identity quaternion) as we only calculated position
            centroid_pose_camera.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            self.get_logger().info(f"centroid estimated in '{source_frame}': [{centroid[0]:.3f}, {centroid[1]:.3f}, {centroid[2]:.3f}]")

            # 4. transform centroid pose to the world frame (e.g., 'odom')
            self.get_logger().info(f"transforming pose from '{source_frame}' to '{self.world_frame_}'...")
            transform_timeout = Duration(seconds=1.0) # max time to wait for transform
            try:
                centroid_pose_odom = self.tf_buffer_.transform(
                    centroid_pose_camera, # the pose to transform
                    self.world_frame_,     # the target frame
                    timeout=transform_timeout # timeout duration
                )
                self.get_logger().info("transform to world frame successful.")
            except TransformException as ex:
                self.get_logger().error(f"could not transform '{source_frame}' to '{self.world_frame_}': {ex}")
                response.success = False
                response.message = f"transform error: {ex}"
                return response

            # the transformed pose in the world frame will be used for marker and collision object
            object_center_pose_odom = centroid_pose_odom

            # 5. publish marker for visualization
            self.get_logger().info("publishing visualization marker...")
            marker = Marker()
            marker.header.frame_id = self.world_frame_ # marker is in the world frame
            marker.header.stamp = now.to_msg() # use the time the point cloud was processed
            marker.ns = "detected_object" # namespace for the marker
            marker.id = 0 # unique id within the namespace
            marker.type = Marker.CUBE # represent object as a cube
            marker.action = Marker.ADD # add or modify the marker
            marker.pose = object_center_pose_odom.pose # set marker pose to the calculated centroid pose
            # set marker scale based on estimated object dimensions parameter
            marker.scale = Vector3(x=self.object_dims_[0], y=self.object_dims_[1], z=self.object_dims_[2])
            marker.color = self.marker_color_ # set marker color from parameters
            marker.lifetime = Duration(seconds=0).to_msg() # marker persists until replaced or deleted (0=infinite)
            self.marker_pub_.publish(marker)
            self.get_logger().info("marker published.")

            # 6. publish collision object for moveit
            self.get_logger().info("publishing collision object...")
            co = CollisionObject()
            co.header.frame_id = self.world_frame_ # collision object is in the world frame
            co.header.stamp = now.to_msg()
            co.id = "target_object_box" # unique id for the collision object
            co.operation = CollisionObject.ADD # add or modify the object in the planning scene
            # define the shape of the collision object
            primitive = SolidPrimitive()
            primitive.type = SolidPrimitive.BOX # use a box shape
            # set dimensions based on parameters, adding padding for safety margin
            primitive.dimensions = [
                self.object_dims_[0] + 2 * self.collision_padding_,
                self.object_dims_[1] + 2 * self.collision_padding_,
                self.object_dims_[2] + 2 * self.collision_padding_
            ]
            co.primitives.append(primitive) # add the shape to the collision object
            # set the pose of the shape (relative to the header frame)
            co.primitive_poses.append(object_center_pose_odom.pose)
            self.collision_pub_.publish(co)
            self.get_logger().info("collision object published.")

            # 7. transform centroid pose from world frame to robot base frame
            self.get_logger().info(f"transforming pose from '{self.world_frame_}' to '{self.robot_base_frame_}'...")
            # ensure the pose to be transformed is indeed a PoseStamped object
            if not isinstance(object_center_pose_odom, PoseStamped):
                 raise TypeError("internal error: object_center_pose_odom is not of type PoseStamped before transform")

            # explicitly set the timestamp to Time(0) again before this transform
            # this ensures tf2 uses the latest available transform between world and base
            object_center_pose_odom.header.stamp = rclpy.time.Time(seconds=0, nanoseconds=0).to_msg()

            try:
                object_pose_base_link_stamped = self.tf_buffer_.transform(
                    object_center_pose_odom,    # the pose in the world frame
                    self.robot_base_frame_,     # the target frame (robot base)
                    timeout=transform_timeout   # timeout duration
                )
                self.get_logger().info("transform to robot base frame successful.")
            except TransformException as ex:
                self.get_logger().error(f"could not transform '{self.world_frame_}' to '{self.robot_base_frame_}': {ex}")
                response.success = False
                response.message = f"transform error: {ex}"
                return response

            # 8. publish the object's pose relative to the robot base frame
            self.get_logger().info("publishing object pose relative to base link...")
            self.object_pose_pub_.publish(object_pose_base_link_stamped)
            self.get_logger().info("object pose in base link published.")

            # 9. set success response for the service call
            response.success = True
            response.message = "object centroid pose captured and published successfully."
            self.get_logger().info("service call finished successfully.")

        # handle specific expected exceptions like transform errors or timeouts
        except (TransformException, TimeoutError) as ex:
            self.get_logger().error(f"error during capture process: {ex}")
            response.success = False
            response.message = f"error: {ex}"
        # handle any other unexpected errors
        except Exception as e:
            self.get_logger().error(f"unexpected error during object capture: {e}", exc_info=True) # log traceback
            response.success = False
            response.message = f"unexpected error: {e}"

        # return the service response (success/failure and message)
        return response

    # --- helper function to wait for a single message ---
    # creates a temporary node and subscriber to wait for one message on a topic
    def wait_for_message(self, msg_type, topic, timeout_sec):
        # create a temporary, minimal node for the subscription
        # use_global_arguments=false prevents interference with the main node's arguments
        temp_node = rclpy.create_node(f'{self.get_name()}_wait_for_message_sub', use_global_arguments=False)
        try:
            msg = None # variable to store the received message
            future = rclpy.Future() # future object to signal completion

            # callback function for the temporary subscriber
            def callback(received_msg):
                nonlocal msg, future
                # ensure we only set the result once
                if not future.done():
                    msg = received_msg
                    future.set_result(True) # signal that a message was received

            # use a best-effort qos profile, keeping only the last message, suitable for sensor data
            # adjust qos if reliability is critical and publisher supports it
            qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
            # create the temporary subscription
            sub = temp_node.create_subscription(msg_type, topic, callback, qos_profile)

            # spin the temporary node until the future is complete or timeout occurs
            rclpy.spin_until_future_complete(temp_node, future, timeout_sec=timeout_sec)

            # clean up the subscription
            temp_node.destroy_subscription(sub)

            # check if the future completed successfully (meaning a message was received)
            if not future.done() or future.result() is not True:
                 self.get_logger().warn(f"timeout waiting for message on {topic} after {timeout_sec} seconds.")
                 return None # return none if timed out

            # return the received message
            return msg
        finally:
            # ensure the temporary node is always destroyed
            temp_node.destroy_node()

# --- main execution function ---
def main(args=None):
    # initialize rclpy library
    rclpy.init(args=args)
    node = None # initialize node variable
    try:
        # create an instance of the node class
        node = ObjectSnapshotNode()
        # keep the node running, processing callbacks (services, timers, subscriptions)
        rclpy.spin(node)
    except KeyboardInterrupt:
        # handle ctrl+c gracefully
        pass
    except ExternalShutdownException:
        # handle external shutdown requests
        pass
    finally:
        # cleanup resources
        if node:
            # explicitly destroy the node
            node.destroy_node()
        if rclpy.ok():
            # shutdown rclpy library
            rclpy.shutdown()

# standard python entry point
if __name__ == '__main__':
    main()
