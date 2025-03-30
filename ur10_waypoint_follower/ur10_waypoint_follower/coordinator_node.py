import rclpy
import csv
from rclpy.node import Node
from rclpy.action import ActionClient
import tf2_ros
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, PositionConstraint, OrientationConstraint, CollisionObject, PlanningScene
from shape_msgs.msg import SolidPrimitive

class WaypointCoordinator(Node):
    def __init__(self):
        super().__init__('waypoint_coordinator')
        # initialize csv_path and offset parameters, can change parameters in launch file
        self.declare_parameter('csv_path', '')
        self.declare_parameter('offset', 0.00)
        self.csv_path = self.get_parameter('csv_path').value
        self.offset = self.get_parameter('offset').value

        # look up current end effector pose
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # action client for moveGroup
        self._action_client = ActionClient(self, MoveGroup, '/move_action')
        self._send_goal_future = None
        self._get_result_future = None

        # ensure sequential execution and wait 3 seconds
        self.is_executing = False
        self.create_timer(3.0, self.run_cycle) # wait 3 sec

        # scene publisher and ground plane setup
        self.scene_publisher = self.create_publisher(PlanningScene, '/planning_scene', 10)
        self.add_ground_plane() # ground plane to avoid robot planning through the floor

    def add_ground_plane(self):
        ground = CollisionObject()
        ground.id = "ground"
        ground.header.frame_id = "base_link"
        ground.operation = CollisionObject.ADD

        # create floor box
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [4.0, 4.0, 0.02]

        ground_pose = PoseStamped()
        ground_pose.header.frame_id = "base_link"
        ground_pose.pose.position.z = -0.03

        ground.primitives.append(box)
        ground.primitive_poses.append(ground_pose.pose)

        scene = PlanningScene()
        scene.world.collision_objects.append(ground)
        scene.is_diff = True
        self.scene_publisher.publish(scene)
        self.get_logger().info("Added enhanced ground collision plane")

    def run_cycle(self):
        # if previous cycle is executing then do nothing
        if self.is_executing:
            return
        self.is_executing = True

        #-------------------------------------------------------------
        # --- get current pose ---
        #-------------------------------------------------------------
        
        try:
            # verify TF frame exists
            if not self.tf_buffer.can_transform('base_link', 'wrist_3_link', rclpy.time.Time()):
                self.get_logger().warn("TF frames not available!")
                self.is_executing = False
                return
            
            trans = self.tf_buffer.lookup_transform('base_link', 'wrist_3_link', rclpy.time.Time())
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {str(e)}")
            self.is_executing = False
            return

        current_pose = Pose()
        current_pose.position.x = trans.transform.translation.x
        current_pose.position.y = trans.transform.translation.y
        current_pose.position.z = trans.transform.translation.z
        current_pose.orientation = trans.transform.rotation

        #-------------------------------------------------------------
        # --- compute offset pose ---
        #-------------------------------------------------------------

        #TO DO : add offset to be random rather than just +5 cm, maybe
        # to something where we could + and - random numbers within a range
        # to ensure the offset remains in the offset boundaries at all times
        # therefore preventing the robot to break
        offset_pose = Pose()
        offset_pose.position.x = current_pose.position.x + self.offset
        offset_pose.position.y = current_pose.position.y
        offset_pose.position.z = current_pose.position.z  
        offset_pose.orientation = current_pose.orientation

        # workspace boundaries check with safety margin
        SAFE_Z_MIN = 0.35  # 0.35cm safety margin
        SAFE_Z_MAX = 1.5  # 1.5m safety amrgin
        if not (SAFE_Z_MIN <= offset_pose.position.z <= SAFE_Z_MAX):
            self.get_logger().warn(f"Offset pose Z ({offset_pose.position.z:.2f}m) out of safe bounds!")
            self.is_executing = False
            return

        #-------------------------------------------------------------
        # --- write current and offset poses to csv ---
        #-------------------------------------------------------------

        try:
            with open(self.csv_path, 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow(['x','y','z','ox','oy','oz','ow'])
                # write current pose
                writer.writerow([
                    current_pose.position.x,
                    current_pose.position.y,
                    current_pose.position.z,
                    current_pose.orientation.x,
                    current_pose.orientation.y,
                    current_pose.orientation.z,
                    current_pose.orientation.w
                ])
                # write offset pose
                writer.writerow([
                    offset_pose.position.x,
                    offset_pose.position.y,
                    offset_pose.position.z,
                    offset_pose.orientation.x,
                    offset_pose.orientation.y,
                    offset_pose.orientation.z,
                    offset_pose.orientation.w
                ])
        except Exception as e:
            self.get_logger().error(f"CSV write error: {str(e)}")
            self.is_executing = False
            return
        self.get_logger().info(f"Wrote poses to CSV at {self.csv_path}")

        #-------------------------------------------------------------
        # --- parse csv to get offset pose ---
        #-------------------------------------------------------------

        parsed_pose = Pose()
        try:
            with open(self.csv_path, 'r') as csvfile:
                lines = list(csv.reader(csvfile))
                if len(lines) >= 3:
                    row = lines[2]
                    parsed_pose.position.x = float(row[0])
                    parsed_pose.position.y = float(row[1])
                    parsed_pose.position.z = float(row[2])
                    parsed_pose.orientation.x = float(row[3])
                    parsed_pose.orientation.y = float(row[4])
                    parsed_pose.orientation.z = float(row[5])
                    parsed_pose.orientation.w = float(row[6])
                else:
                    raise ValueError("Insufficient CSV rows")
        except Exception as e:
            self.get_logger().error(f"CSV parse error: {str(e)}")
            self.is_executing = False
            return

        # final safety check before sending goal
        if parsed_pose.position.z < SAFE_Z_MIN:
            self.get_logger().error(f"DANGER! Parsed Z ({parsed_pose.position.z:.2f}m) below minimum safety threshold!")
            self.is_executing = False
            return

        self.get_logger().info("Validated parsed pose, sending goal to robot.")
        self.send_goal(parsed_pose)

    def send_goal(self, pose):
        # verify action server is available
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Action server not available!")
            self.is_executing = False
            return

        # configure motion plan request
        goal_msg = MoveGroup.Goal()
        request = MotionPlanRequest()
        request.group_name = "ur_manipulator"
        request.num_planning_attempts = 20
        request.allowed_planning_time = 20.0
        request.planner_id = "RRTConnect"
        request.max_velocity_scaling_factor = 0.4
        request.max_acceleration_scaling_factor = 0.2

        # position constraint using box
        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = "base_link"
        pos_constraint.link_name = "wrist_3_link"
        pos_constraint.weight = 1.0
        
        volume = SolidPrimitive()
        volume.type = SolidPrimitive.BOX 
        volume.dimensions = [0.02, 0.02, 0.02] 
        
        position_pose = Pose()
        position_pose.position = pose.position
        position_pose.orientation.w = 1.0  # maintain orientation consistency
        
        pos_constraint.constraint_region.primitives.append(volume)
        pos_constraint.constraint_region.primitive_poses.append(position_pose)

        # orientation constraint
        orient_constraint = OrientationConstraint()
        orient_constraint.header.frame_id = "base_link"
        orient_constraint.link_name = "wrist_3_link"
        orient_constraint.orientation = pose.orientation
        orient_constraint.absolute_x_axis_tolerance = 0.05
        orient_constraint.absolute_y_axis_tolerance = 0.05
        orient_constraint.absolute_z_axis_tolerance = 0.05
        orient_constraint.weight = 1.0

        # combine constraints
        constraints = Constraints()
        constraints.position_constraints.append(pos_constraint)
        constraints.orientation_constraints.append(orient_constraint)
        request.goal_constraints.append(constraints)
        goal_msg.request = request

        try:
            self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        except Exception as e:
            self.get_logger().error(f"Goal send failed: {str(e)}")
            self.is_executing = False
            return

        if self._send_goal_future is not None:
            self._send_goal_future.add_done_callback(self.goal_response_callback)
        else:
            self.get_logger().error("Goal future is None!")
            self.is_executing = False

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by server')
            self.is_executing = False
            return

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        if result.error_code.val == result.error_code.SUCCESS:
            self.get_logger().info('Motion completed successfully!')
        else:
            self.get_logger().error(f'Motion failed with error code: {result.error_code.val}')
        self.is_executing = False

def main(args=None):
    rclpy.init(args=args)
    node = WaypointCoordinator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()