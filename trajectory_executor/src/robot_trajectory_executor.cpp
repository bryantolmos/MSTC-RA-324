#include <memory>
#include <vector>
#include <string>
#include <iostream>
#include <thread>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
// moveit includes
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit/robot_state/conversions.h>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose.hpp>
// message/service includes
#include <geometry_msgs/msg/pose_array.hpp>
#include <std_srvs/srv/trigger.hpp>

typedef std_srvs::srv::Trigger Trigger;

class RobotTrajectoryExecutor : public rclcpp::Node
{
public:
  RobotTrajectoryExecutor(const rclcpp::NodeOptions &options)
  : Node("robot_trajectory_executor", options)
  {
    if (!this->has_parameter("planning_group")) {
      this->declare_parameter<std::string>("planning_group", "ur_manipulator");
    }
    if (!this->has_parameter("waypoint_topic")) {
      this->declare_parameter<std::string>("waypoint_topic", "/planned_waypoints");
    }
    if (!this->has_parameter("execute_service_name")) {
      this->declare_parameter<std::string>("execute_service_name", "/execute_web_trajectory");
    }
    if (!this->has_parameter("max_waypoints")) {
      this->declare_parameter<int>("max_waypoints", 50);
    }
    if (!this->has_parameter("floor_check_z")) {
      this->declare_parameter<double>("floor_check_z", -0.05);
    }
    if (!this->has_parameter("robot_base_frame")) {
      this->declare_parameter<std::string>("robot_base_frame", "base_link");
    }
    if (!this->has_parameter("waypoint_offset_z")) {
      this->declare_parameter<double>("waypoint_offset_z", 0.05); // Default offset of 5 cm
    }

    planning_group_    = this->get_parameter("planning_group").as_string();
    waypoint_topic_   = this->get_parameter("waypoint_topic").as_string();
    execute_service_  = this->get_parameter("execute_service_name").as_string();
    max_waypoints_    = this->get_parameter("max_waypoints").as_int();
    floor_check_z_    = this->get_parameter("floor_check_z").as_double();
    robot_base_frame_ = this->get_parameter("robot_base_frame").as_string();
    waypoint_offset_z_= this->get_parameter("waypoint_offset_z").as_double();

    RCLCPP_INFO(get_logger(), "Using planning_group: %s", planning_group_.c_str());
    RCLCPP_INFO(get_logger(), "Robot base frame: %s", robot_base_frame_.c_str());
    RCLCPP_INFO(get_logger(), "Listening to waypoint topic: %s", waypoint_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Execution service name: %s", execute_service_.c_str());
    RCLCPP_INFO(get_logger(), "Max waypoints limit: %d", max_waypoints_);
    RCLCPP_INFO(get_logger(), "Floor check Z threshold: %.3f", floor_check_z_);
    RCLCPP_INFO(get_logger(), "Waypoint Z offset: %.3f", waypoint_offset_z_);

    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*){}),
      planning_group_
    );
    move_group_->setPlannerId("LIN");
    if (move_group_->getPlanningFrame().empty()) {
      RCLCPP_ERROR(get_logger(), "Failed to initialize MoveGroupInterface for group '%s'. Is MoveIt running?", planning_group_.c_str());
      throw std::runtime_error("MoveGroupInterface initialization failed");
    }

    planning_scene_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();

    // --- Inject floor collision object into MoveIt planning scene ---
    {
      moveit_msgs::msg::CollisionObject floor_co;
      floor_co.header.frame_id = robot_base_frame_;
      floor_co.id = "floor";

      shape_msgs::msg::SolidPrimitive floor_primitive;
      floor_primitive.type = floor_primitive.BOX;
      floor_primitive.dimensions = {2.0, 2.0, 0.01};

      geometry_msgs::msg::Pose floor_pose;
      floor_pose.orientation.w = 1.0;
      floor_pose.position.x = 0.0;
      floor_pose.position.y = 0.0;
      floor_pose.position.z = -0.02;

      floor_co.primitives.push_back(floor_primitive);
      floor_co.primitive_poses.push_back(floor_pose);

      std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
      collision_objects.push_back(floor_co);
      planning_scene_->applyCollisionObjects(collision_objects);

      RCLCPP_INFO(get_logger(), "Injected floor collision object into MoveIt planning scene.");
    }

    display_pub_ = create_publisher<moveit_msgs::msg::DisplayTrajectory>(
      "/display_planned_path", 10
    );

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    waypoint_sub_ = create_subscription<geometry_msgs::msg::PoseArray>(
      waypoint_topic_, qos,
      std::bind(&RobotTrajectoryExecutor::waypointCallback, this, std::placeholders::_1)
    );

    exec_srv_ = create_service<Trigger>(
      execute_service_,
      std::bind(&RobotTrajectoryExecutor::executeCallback, this, std::placeholders::_1, std::placeholders::_2)
    );

    RCLCPP_INFO(get_logger(), "RobotTrajectoryExecutor ready.");
  }

private:
  void waypointCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
  {
    latest_waypoints_ = msg->poses;
    RCLCPP_INFO(get_logger(), "Received %zu waypoints.", latest_waypoints_.size());

    for (auto& pose : latest_waypoints_) {
      pose.position.z += waypoint_offset_z_;
      if (pose.position.z < floor_check_z_) {
        RCLCPP_WARN(get_logger(), "Waypoint Z coordinate (%.3f) is below floor check threshold (%.3f).",
                    pose.position.z, floor_check_z_);
      }
    }
  }

  void executeCallback(
    const std::shared_ptr<Trigger::Request> /*request*/,
    std::shared_ptr<Trigger::Response> res)
  {
    if (latest_waypoints_.empty())
    {
      RCLCPP_ERROR(get_logger(), "Execute called but no waypoints received.");
      res->success = false;
      res->message = "No waypoints available to execute.";
      return;
    }
    if (latest_waypoints_.size() > static_cast<size_t>(max_waypoints_))
    {
       RCLCPP_ERROR(get_logger(), "Too many waypoints received (%zu > %d).", latest_waypoints_.size(), max_waypoints_);
      res->success = false;
      res->message = "Number of waypoints exceeds maximum limit.";
      return;
    }

    moveit_msgs::msg::RobotTrajectory trajectory;
    double fraction = move_group_->computeCartesianPath(latest_waypoints_, 0.01, 0.0, trajectory);

    if (fraction < 0.9)
    {
      RCLCPP_ERROR(get_logger(), "Cartesian path planning failed or could not achieve sufficient fraction (%.2f).", fraction);
      res->success = false;
      res->message = "Cartesian path planning failed.";
      return;
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;

    moveit_msgs::msg::DisplayTrajectory disp;
    moveit::core::robotStateToRobotStateMsg(*move_group_->getCurrentState(), disp.trajectory_start);
    disp.trajectory.push_back(plan.trajectory_);
    display_pub_->publish(disp);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    std::cout << "Plan successful. Execute trajectory? (y/N): ";
    std::string user_input;
    std::getline(std::cin, user_input);

    if (user_input != "y" && user_input != "Y") {
      RCLCPP_WARN(get_logger(), "Execution aborted by user.");
      res->success = false;
      res->message = "Execution aborted by user.";
      return;
    }

    RCLCPP_INFO(get_logger(), "Executing trajectory...");
    moveit::core::MoveItErrorCode execute_result = move_group_->execute(plan.trajectory_);

    if (execute_result == moveit::core::MoveItErrorCode::SUCCESS) {
        res->success = true;
        res->message = "Execution succeeded.";
        RCLCPP_INFO(get_logger(), "Execution completed successfully.");
    } else {
        res->success = false;
        res->message = "Execution failed with error code: " + std::to_string(execute_result.val);
        RCLCPP_ERROR(get_logger(), "Execution failed with error code %d", execute_result.val);
    }
  }

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_;

  rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>::SharedPtr display_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr waypoint_sub_;
  rclcpp::Service<Trigger>::SharedPtr exec_srv_;

  std::string planning_group_, waypoint_topic_, execute_service_, robot_base_frame_;
  int max_waypoints_;
  double floor_check_z_;
  double waypoint_offset_z_;
  std::vector<geometry_msgs::msg::Pose> latest_waypoints_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RobotTrajectoryExecutor>(
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(false)
  );
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
