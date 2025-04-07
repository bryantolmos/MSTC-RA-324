import sys

# rclpy imports
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.executors import ExternalShutdownException

# tf2 imports
import tf2_ros
from tf2_ros import TransformException, Buffer, TransformListener
# needed for transforming geometry_msgs types
import tf2_geometry_msgs

# ROS message imports
from zed_msgs.msg import ObjectsStamped
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion, Vector3
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
# import the message type for duration fields
from builtin_interfaces.msg import Duration as DurationMsg


class ObjectProcessorNode(Node):
    """
    processes detected objects from a zed camera.

    subscribes to objectsstamped messages, filters them based on confidence,
    transforms their poses to a target frame, and publishes:
    1. moveit_msgs/collisionobject messages for motion planning.
    2. visualization_msgs/markerarray messages for rviz visualization.
    """
    def __init__(self):
        super().__init__('object_processor_node')

        # declare node parameters and get their values
        self._declare_and_get_parameters()

        # log parameters for verification
        self._log_parameters()

        # setup tf2 listener and buffer
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # setup publishers
        self.collision_pub = self.create_publisher(
            CollisionObject,
            '~/collision_objects', # topic relative to node's namespace
            10)
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '~/detection_markers', # topic relative to node's namespace
            10)

        # setup subscriber
        self.object_detect_sub = self.create_subscription(
            ObjectsStamped,
            self.object_detect_topic, # topic name from parameter
            self.object_detect_callback,
            10) # using default qos depth 10

        # keep track of objects published in the last cycle for deletion handling
        self.last_published_ids = set()

        self.get_logger().info(f"{self.get_name()} started successfully.")

    def _declare_and_get_parameters(self):
        """declare and read node parameters."""
        self.declare_parameter('object_detect_topic', '/zed/zed_node/obj_det/objects')
        self.declare_parameter('target_frame', 'odom')
        self.declare_parameter('confidence_threshold', 0.6)
        self.declare_parameter('collision_padding', 0.05) # meters
        self.declare_parameter('marker_lifetime_s', 1.0) # seconds
        self.declare_parameter('marker_color_r', 0.0)
        self.declare_parameter('marker_color_g', 1.0)
        self.declare_parameter('marker_color_b', 0.0)
        self.declare_parameter('marker_color_a', 0.5) # semi-transparent

        self.object_detect_topic = self.get_parameter('object_detect_topic').get_parameter_value().string_value
        self.target_frame = self.get_parameter('target_frame').get_parameter_value().string_value
        self.confidence_thresh = self.get_parameter('confidence_threshold').get_parameter_value().double_value
        self.collision_padding = self.get_parameter('collision_padding').get_parameter_value().double_value
        self.marker_lifetime_rclpy = Duration(seconds=self.get_parameter('marker_lifetime_s').get_parameter_value().double_value)
        self.marker_color = ColorRGBA(
            r=self.get_parameter('marker_color_r').get_parameter_value().double_value,
            g=self.get_parameter('marker_color_g').get_parameter_value().double_value,
            b=self.get_parameter('marker_color_b').get_parameter_value().double_value,
            a=self.get_parameter('marker_color_a').get_parameter_value().double_value
        )

    def _log_parameters(self):
        """log the parameters."""
        self.get_logger().info(f"subscribing to objectsstamped on: {self.object_detect_topic}")
        self.get_logger().info(f"publishing collisionobjects to: {self.get_namespace()}/collision_objects")
        self.get_logger().info(f"publishing markerarray to: {self.get_namespace()}/detection_markers")
        self.get_logger().info(f"target frame for outputs: {self.target_frame}")
        self.get_logger().info(f"confidence threshold: {self.confidence_thresh}")
        self.get_logger().info(f"collision padding: {self.collision_padding} m")
        self.get_logger().info(f"marker lifetime: {self.marker_lifetime_rclpy.nanoseconds / 1e9} s") # log the value

    def object_detect_callback(self, msg: ObjectsStamped):
        """process detected objects messages."""
        self.get_logger().debug("object detection callback triggered.")
        now = self.get_clock().now()
        marker_array = MarkerArray()
        current_detected_ids = set() # track ids detected in this message
        source_frame = msg.header.frame_id

        # check if the transform is available at all before processing objects
        try:
            # check transform availability without timeout first
            self.tf_buffer.lookup_transform(self.target_frame, source_frame, rclpy.time.Time())
        except TransformException as ex:
            # log only periodically to avoid flooding
            self.get_logger().warn(
                f"cannot transform from '{source_frame}' to target_frame '{self.target_frame}'. skipping message. error: {ex}",
                throttle_duration_sec=5.0)
            return

        # process each detected object
        for obj in msg.objects:
            # filter low confidence detections
            if obj.confidence < self.confidence_thresh:
                continue

            # create a unique id for collision object and potentially markers
            collision_object_id = f"{obj.label.lower()}_{obj.label_id}"
            current_detected_ids.add(collision_object_id)

            # prepare pose for transformation
            original_pose_msg = Pose()
            original_pose_msg.position.x = float(obj.position[0]) # cast to float
            original_pose_msg.position.y = float(obj.position[1]) # cast to float
            original_pose_msg.position.z = float(obj.position[2]) # cast to float
            original_pose_msg.orientation.w = 1.0 # assume axis-aligned
            original_pose_msg.orientation.x = 0.0
            original_pose_msg.orientation.y = 0.0
            original_pose_msg.orientation.z = 0.0

            # create a geometry_msgs/posestamped for the transform function
            original_pose_stamped = PoseStamped()
            original_pose_stamped.header = msg.header # use header from input message
            original_pose_stamped.pose = original_pose_msg

            # transform pose to target frame
            try:
                transformed_pose_stamped = self.tf_buffer.transform(
                    original_pose_stamped,
                    self.target_frame,
                    timeout=Duration(seconds=0.1)
                )
                transformed_pose = transformed_pose_stamped.pose # extract pose part

            except TransformException as ex:
                self.get_logger().warn(
                    f"failed to transform pose for object {collision_object_id} from "
                    f"{source_frame} to {self.target_frame}: {ex}",
                    throttle_duration_sec=5.0)
                continue # skip this object

            # publish collision object for moveit
            collision_object = CollisionObject()
            collision_object.header.stamp = now.to_msg()
            collision_object.header.frame_id = self.target_frame
            collision_object.id = collision_object_id
            collision_object.operation = CollisionObject.ADD

            primitive = SolidPrimitive()
            primitive.type = SolidPrimitive.BOX
            # cast dimensions to float and add padding
            primitive.dimensions = [
                max(0.01, float(obj.dimensions_3d[0]) + 2 * self.collision_padding), 
                max(0.01, float(obj.dimensions_3d[1]) + 2 * self.collision_padding),
                max(0.01, float(obj.dimensions_3d[2]) + 2 * self.collision_padding)
            ]

            collision_object.primitives.append(primitive)
            collision_object.primitive_poses.append(transformed_pose)

            self.get_logger().debug(f"attempting to publish collisionobject id: {collision_object_id}")
            self.collision_pub.publish(collision_object)

            # create visualization marker
            marker_cube = Marker()
            marker_cube.header.stamp = now.to_msg()
            marker_cube.header.frame_id = self.target_frame
            marker_cube.ns = "detected_objects_cubes"
            marker_cube.id = obj.label_id # using label_id as marker id
            marker_cube.type = Marker.CUBE
            marker_cube.action = Marker.ADD
            marker_cube.pose = transformed_pose
            # cast dimensions to float for scale
            marker_cube.scale = Vector3( # create vector3 directly
                 x=max(0.01, float(obj.dimensions_3d[0])), 
                 y=max(0.01, float(obj.dimensions_3d[1])),
                 z=max(0.01, float(obj.dimensions_3d[2]))
            )
            marker_cube.color = self.marker_color
            # convert rclpy duration to message duration
            marker_cube.lifetime = self.marker_lifetime_rclpy.to_msg()

            marker_array.markers.append(marker_cube)

            # create visualization marker
            marker_text = Marker()
            marker_text.header = marker_cube.header
            marker_text.ns = "detected_objects_labels"
            marker_text.id = marker_cube.id
            marker_text.type = Marker.TEXT_VIEW_FACING
            marker_text.action = Marker.ADD
            marker_text.pose = transformed_pose
            # cast dimension used for z offset
            marker_text.pose.position.z += (float(obj.dimensions_3d[2]) / 2.0) + 0.15
            marker_text.scale.z = 0.15 # text height
            marker_text.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.95)
            marker_text.text = f"{obj.label} ({obj.confidence:.2f})"
            # convert rclpy duration to message duratio
            marker_text.lifetime = self.marker_lifetime_rclpy.to_msg()

            marker_array.markers.append(marker_text)

        # handle deletions
        disappeared_ids = self.last_published_ids - current_detected_ids

        for disappeared_id in disappeared_ids:
            # publish remove for collision object
            remove_co = CollisionObject()
            remove_co.header.stamp = now.to_msg()
            remove_co.header.frame_id = self.target_frame
            remove_co.id = disappeared_id
            remove_co.operation = CollisionObject.REMOVE
            self.collision_pub.publish(remove_co)
            self.get_logger().debug(f"publishing remove collisionobject id: {disappeared_id}")

            # publish delete for markers
            try:
                marker_id_int = int(disappeared_id.split('_')[-1])
            except (ValueError, IndexError):
                 self.get_logger().warn(f"could not parse marker id from collision id '{disappeared_id}' for deletion")
                 continue

            marker_delete_cube = Marker()
            marker_delete_cube.header.stamp = now.to_msg()
            marker_delete_cube.header.frame_id = self.target_frame
            marker_delete_cube.ns = "detected_objects_cubes"
            marker_delete_cube.id = marker_id_int
            marker_delete_cube.action = Marker.DELETE

            marker_array.markers.append(marker_delete_cube)

            marker_delete_text = Marker()
            marker_delete_text.header = marker_delete_cube.header
            marker_delete_text.ns = "detected_objects_labels"
            marker_delete_text.id = marker_id_int
            marker_delete_text.action = Marker.DELETE
            marker_array.markers.append(marker_delete_text)

        # publish marker array
        if marker_array.markers:
            self.marker_pub.publish(marker_array)

        # update state for next cycle
        self.last_published_ids = current_detected_ids

    def destroy_node(self):
        """cleanup on shutdown."""
        self.get_logger().info("shutting down, publishing deleteall markers.")
        marker_array = MarkerArray()
        # create a single marker message with deleteall action for each namespace
        delete_all_cubes = Marker(action=Marker.DELETEALL, ns="detected_objects_cubes")
        delete_all_labels = Marker(action=Marker.DELETEALL, ns="detected_objects_labels")
        marker_array.markers.append(delete_all_cubes)
        marker_array.markers.append(delete_all_labels)

        # publish the deleteall message
        try:
            self.marker_pub.publish(marker_array)
        except Exception as e:
            self.get_logger().error(f"exception during shutdown marker publish: {e}")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ObjectProcessorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass # allow clean shutdown on ctrl-c
    except ExternalShutdownException:
        sys.exit(1) # handle external shutdown request
    finally:
        if node:
            # cleanup happens in node.destroy_node() override
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()

