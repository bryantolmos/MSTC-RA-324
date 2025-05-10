from geometry_msgs.msg import PoseStamped, Pose
from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.action import MoveGroup
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
import rclpy
from rclpy.node import Node
import rclpy.duration
from builtin_interfaces.msg import Duration  # Import Duration from builtin_interfaces
import time
import math  # Import the math library

class WaypointCoordinator(Node):
    def __init__(self):
        super().__init__('waypoint_coordinator')
        self.cartesian_path_client = self.create_client(GetCartesianPath, '/compute_cartesian_path')
        while not self.cartesian_path_client.wait_for_service(timeout_sec=5.0): # Increase timeout
            self.get_logger().info('Waiting for /compute_cartesian_path service...')
        self.move_group_name = "ur_manipulator"  # Adjust if your move group name is different
        self.planning_frame = "base_link"        # Adjust if your planning frame is different
        self.eef_link = "tool0"                  # Adjust if your end-effector link is different
        # Correct the action server name here:
        self.trajectory_action_client = ActionClient(self, FollowJointTrajectory, '/joint_trajectory_controller/follow_joint_trajectory')
        self._goal_future = None
        self._get_result_future = None
        self.action_server_available = False # Add this line

    def create_waypoint(self, x, y, z, qx=0.0, qy=0.0, qz=0.0, qw=1.0):
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw
        return pose

    def quaternion_from_euler(self, roll, pitch, yaw):
        """
        Converts Euler angles (roll, pitch, yaw) to a quaternion.
        This function assumes the commonly used Tait-Bryan convention
        where the rotations are applied in the order: yaw (Z), pitch (Y), roll (X).
        Args:
            roll (float): Rotation around the X-axis (in radians).
            pitch (float): Rotation around the Y-axis (in radians).
            yaw (float): Rotation around the Z-axis (in radians).

        Returns:
            tuple: A tuple (qx, qy, qz, qw) representing the quaternion.
        """
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy

        return qx, qy, qz, qw

    def send_cartesian_waypoints(self):
        self.get_logger().info("Sending Cartesian waypoints...")
        request = GetCartesianPath.Request()
        request.header.frame_id = self.planning_frame
        request.group_name = self.move_group_name
        request.avoid_collisions = True
        request.max_step = 0.02
        waypoints = []

        # 1. Start Pose (Tool facing down)
        start_pose = self.create_waypoint(0.6, 0.0, 0.5, qx=0.0, qy=0.0, qz=0.0, qw=1.0)
        waypoints.append(start_pose)

        # 2. Move Down
        down_pose = self.create_waypoint(0.6, 0.0, 0.3, qx=0.0, qy=0.0, qz=0.0, qw=1.0)
        waypoints.append(down_pose)

        # 3. Rotate End-Effector Forward
        roll_forward = 0.0
        pitch_forward = math.pi / 2  # 90 degrees around Y
        yaw_forward = 0.0
        qx_forward, qy_forward, qz_forward, qw_forward = self.quaternion_from_euler(roll_forward, pitch_forward, yaw_forward)
        forward_pose_1 = self.create_waypoint(0.6, 0.0, 0.3, qx=qx_forward, qy=qy_forward, qz=qz_forward, qw=qw_forward)
        waypoints.append(forward_pose_1)

        # 4. Move Right
        right_pose = self.create_waypoint(0.6, 0.8, 0.3, qx=qx_forward, qy=qy_forward, qz=qz_forward, qw=qw_forward)
        waypoints.append(right_pose)

        # 5. Move Back 
        original_pose_2 = self.create_waypoint(0.4, 0.8, 0.3, qx=qx_forward, qy=qy_forward, qz=qz_forward, qw=qw_forward)
        waypoints.append(original_pose_2)

        # 6. Rotate End-Effector Back to Original
        roll_original = 0.0
        pitch_original = 0.0
        yaw_original = 0.0
        qx_original, qy_original, qz_original, qw_original = self.quaternion_from_euler(roll_original, pitch_original, yaw_original)
        original_pose_1 = self.create_waypoint(0.4, 0.8, 0.3, qx=qx_original, qy=qy_original, qz=qz_original, qw=qw_original)
        waypoints.append(original_pose_1)

        # 7. Move Back to Original Position
        original_pose_2 = self.create_waypoint(0.6, 0.0, 0.5, qx=qx_original, qy=qy_original, qz=qz_original, qw=qw_original)
        waypoints.append(original_pose_2)

        request.waypoints = waypoints
        self.future = self.cartesian_path_client.call_async(request)
        self.future.add_done_callback(self.handle_cartesian_path_response)

    def handle_cartesian_path_response(self, future):
        try:
            response = future.result()
            if response.fraction >= 0.99:
                trajectory = response.solution.joint_trajectory
                self.get_logger().info("Cartesian path planned successfully. Now executing...")
                self.execute_trajectory(trajectory)
            else:
                self.get_logger().warn(f"Cartesian path planning failed. Fraction: {response.fraction}")
        except Exception as e:
            self.get_logger().error(f"Service call failed {e}")

    def execute_trajectory(self, trajectory):
        self.get_logger().info("Executing trajectory...")
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = trajectory
        # Create a Duration object, ensuring it's from builtin_interfaces
        goal_msg.goal_time_tolerance = Duration(sec=2, nanosec=0) # Increased tolerance

        if not self.action_server_available: # Check if the action server is available
            self.get_logger().error("Action server is not available!")
            return

        self._goal_future = self.trajectory_action_client.send_goal_async(
            goal_msg,
            feedback_callback=self._feedback_callback)
        self.get_logger().info(f"Goal sent: {goal_msg}")
        self._goal_future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected :(')
            return
        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self._get_result_callback)

    def _get_result_callback(self, future):
        result = future.result()
        self.get_logger().info(f'Result: {result}')
        # rclpy.shutdown()
    def _feedback_callback(self, feedback_msg):
        # You can process feedback here if needed
        pass

    def check_action_server(self):
        """Check if the action server is available."""
        for i in range(5):
            if self.trajectory_action_client.wait_for_server(timeout_sec=1.0):
                self.action_server_available = True
                self.get_logger().info("Action server is available.")
                return
            else:
                self.get_logger().warn("Action server is not available. Retrying...")
                time.sleep(1.0)
        self.get_logger().error("Action server is not available after 5 tries.")
        self.action_server_available = False

def main(args=None):
    rclpy.init(args=args)
    node = WaypointCoordinator()
    node.check_action_server()
    if node.action_server_available:
        node.send_cartesian_waypoints()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()