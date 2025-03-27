import rclpy
import csv
from rclpy.node import Node
from rclpy.action import ActionClient
import tf2_ros
from geometry_msgs.msg import Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint

class WaypointCoordinator(Node):
    def __init__(self):
        super().__init__('waypoint_coordinator')
        # initialize csv_path and offset parameters, can change parameters in launch file
        self.declare_parameter('csv_path', '')
        self.declare_parameter('offset', 0)
        self.csv_path = self.get_parameter('csv_path').value
        self.offset = self.get_parameter('offset').value

        # look up current end effector pose
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # action client for moveGroup
        self._action_client = ActionClient(self, MoveGroup, 'move_action')

        # ensure sequential execution
        self.is_executing = False

        # timer to check every second, but cycle will only start if not busy
        self.timer = self.create_timer(1.0, self.run_cycle)

    def run_cycle(self):
        # if previous cycle is executing then do nothing
        if self.is_executing:
            return
        self.is_executing = True

        #-------------------------------------------------------------
        # --- get current pose ---
        #-------------------------------------------------------------
        
        try:
            trans = self.tf_buffer.lookup_transform(
                'base_link',  # target (robot base)
                'wrist_3_link',    # source (end effector)
                rclpy.time.Time())
        except Exception as e:
            self.get_logger().warn("TF not available yet: " + str(e))
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

        # check workspace boundaries.
        if not (0.3 <= offset_pose.position.z <= 1.5):
            self.get_logger().warn("Offset pose out of safe bounds. Skipping this cycle.")
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
            self.get_logger().error("Error writing CSV: " + str(e))
            self.is_executing = False
            return

        self.get_logger().info(f"Wrote poses to CSV at {self.csv_path}")

        #-------------------------------------------------------------
        # --- parse csv to get offset pose ---
        #-------------------------------------------------------------

        parsed_pose = None
        try:
            with open(self.csv_path, 'r') as csvfile:
                lines = list(csv.reader(csvfile))
                # expecting header then row for current pose then row for offset pose
                if len(lines) >= 3:
                    row = lines[2]
                    parsed_pose = Pose()
                    parsed_pose.position.x = float(row[0])
                    parsed_pose.position.y = float(row[1])
                    parsed_pose.position.z = float(row[2])
                    parsed_pose.orientation.x = float(row[3])
                    parsed_pose.orientation.y = float(row[4])
                    parsed_pose.orientation.z = float(row[5])
                    parsed_pose.orientation.w = float(row[6])
                else:
                    self.get_logger().error("CSV file does not contain enough rows.")
                    self.is_executing = False
                    return
        except Exception as e:
            self.get_logger().error("Error reading CSV: " + str(e))
            self.is_executing = False
            return

        self.get_logger().info("Parsed offset pose from CSV; sending goal to robot.")

        #-------------------------------------------------------------
        # --- send offset pose to rviz ---
        #-------------------------------------------------------------
        self.send_goal(parsed_pose)

    def send_goal(self, pose):
        goal_msg = MoveGroup.Goal()

        # Configure motion plan request.
        request = MotionPlanRequest()
        request.group_name = "ur_manipulator"
        request.num_planning_attempts = 20
        request.allowed_planning_time = 20
        request.planner_id = "RRTConnect"
        request.max_velocity_scaling_factor = 0.5
        request.max_acceleration_scaling_factor = 0.3

        # adding constraints
        joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]
        constraint = Constraints()
        for name in joint_names:
            jc = JointConstraint()
            jc.joint_name = name
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            constraint.joint_constraints.append(jc)
        request.goal_constraints.append(constraint)

        goal_msg.request = request

        # wait for server and send goal
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected by action server.')
            self.is_executing = False
            return

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        if result.error_code.val == result.error_code.SUCCESS:
            self.get_logger().info('Motion plan succeeded!')
        else:
            self.get_logger().error(f'Motion plan failed with error code: {result.error_code.val}')
        # cycle complete then allow the next cycle
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
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
