#include <memory>
#include <vector>
#include <string>
#include <iostream> // for std::cout, std::getline
#include <thread>   // for std::this_thread
#include <chrono>   // for std::chrono

#include <rclcpp/rclcpp.hpp>
// moveit includes
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit/robot_state/conversions.h> // needed for converting robot state to message for display
// message/service includes
#include <geometry_msgs/msg/pose_array.hpp>
#include <std_srvs/srv/trigger.hpp>

// create a shorter alias for the trigger service type
typedef std_srvs::srv::Trigger Trigger;

class RobotTrajectoryExecutor : public rclcpp::Node
{
public:
  RobotTrajectoryExecutor(const rclcpp::NodeOptions &options)
  : Node("robot_trajectory_executor", options)
  {
    // declare parameters only if they haven't been set externally (e.g., via launch file)
    // planning group name used by moveit
    if (!this->has_parameter("planning_group")) {
      this->declare_parameter<std::string>("planning_group", "ur_manipulator");
    }
    // topic to receive waypoints on
    if (!this->has_parameter("waypoint_topic")) {
      this->declare_parameter<std::string>("waypoint_topic", "/planned_waypoints");
    }
    // service name to trigger execution
    if (!this->has_parameter("execute_service_name")) {
      this->declare_parameter<std::string>("execute_service_name", "/execute_web_trajectory");
    }
    // maximum allowed waypoints (safety/sanity check)
    if (!this->has_parameter("max_waypoints")) {
      this->declare_parameter<int>("max_waypoints", 50);
    }
    // add safety check parameter (floor_check_z) - already declared in python launch
     if (!this->has_parameter("floor_check_z")) {
      this->declare_parameter<double>("floor_check_z", -0.05);
    }
     // add base frame parameter - already declared in python launch
     if (!this->has_parameter("robot_base_frame")) {
      this->declare_parameter<std::string>("robot_base_frame", "base_link");
    }

    // retrieve parameter values
    planning_group_  = this->get_parameter("planning_group").as_string();
    waypoint_topic_  = this->get_parameter("waypoint_topic").as_string();
    execute_service_ = this->get_parameter("execute_service_name").as_string();
    max_waypoints_   = this->get_parameter("max_waypoints").as_int();
    // retrieve new parameters (though not used in this provided snippet logic yet)
    floor_check_z_   = this->get_parameter("floor_check_z").as_double();
    robot_base_frame_ = this->get_parameter("robot_base_frame").as_string();


    // log configuration
    RCLCPP_INFO(get_logger(), "Using planning_group: %s", planning_group_.c_str());
    RCLCPP_INFO(get_logger(), "Robot base frame: %s", robot_base_frame_.c_str());
    RCLCPP_INFO(get_logger(), "Listening to waypoint topic: %s", waypoint_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Execution service name: %s", execute_service_.c_str());
    RCLCPP_INFO(get_logger(), "Max waypoints limit: %d", max_waypoints_);
    RCLCPP_INFO(get_logger(), "Floor check Z threshold: %.3f", floor_check_z_);


    // initialize move group interface for the specified planning group
    // use a lambda with empty deleter to prevent shared ownership issues with 'this' node pointer
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*){}),
      planning_group_
    );
    // set the desired planner (pilz linear interpolation planner)
    move_group_->setPlannerId("LIN");
    // check if initialization was successful
    if (move_group_->getPlanningFrame().empty()) {
      RCLCPP_ERROR(get_logger(), "Failed to initialize MoveGroupInterface for group '%s'. Is MoveIt running?", planning_group_.c_str());
      throw std::runtime_error("MoveGroupInterface initialization failed");
    }

    // initialize planning scene interface (optional, used for adding collision objects etc.)
    planning_scene_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();

    // publisher to display the planned trajectory in rviz
    display_pub_ = create_publisher<moveit_msgs::msg::DisplayTrajectory>(
      "/display_planned_path", 10
    );

    // subscriber for receiving the list of waypoints
    // use transient local qos to get the last published waypoint list even if subscribing late
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    waypoint_sub_ = create_subscription<geometry_msgs::msg::PoseArray>(
      waypoint_topic_, qos,
      std::bind(&RobotTrajectoryExecutor::waypointCallback, this, std::placeholders::_1)
    );

    // service server to trigger the planning and execution process
    exec_srv_ = create_service<Trigger>(
      execute_service_,
      std::bind(&RobotTrajectoryExecutor::executeCallback, this, std::placeholders::_1, std::placeholders::_2)
    );

    RCLCPP_INFO(get_logger(), "RobotTrajectoryExecutor ready.");
  }

private:
  // callback function for receiving waypoints
  void waypointCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
  {
    // store the received poses
    latest_waypoints_ = msg->poses;
    RCLCPP_INFO(get_logger(), "Received %zu waypoints.", latest_waypoints_.size());
    // todo: add floor_check_z validation here
  }

  // callback function for the execution trigger service
  void executeCallback(
    const std::shared_ptr<Trigger::Request> /*request*/, // request is unused for Trigger
    std::shared_ptr<Trigger::Response> res)
  {
    // check for valid number of waypoints
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
    // todo: add floor_check_z validation loop here before planning

    // plan a linear trajectory to the last waypoint using the lin planner
    // note: this currently only plans to the *last* waypoint.
    //       for multi-point trajectories, computeCartesianPath should be used.
    move_group_->setPoseTarget(latest_waypoints_.back());
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    // attempt to plan the trajectory
    bool ok = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    // check if planning was successful
    if (!ok) {
      RCLCPP_ERROR(get_logger(), "Pilz LIN planning failed.");
      res->success = false;
      res->message = "Planning failed.";
      return;
    }

    // create and publish a displaytrajectory message for visualization
    moveit_msgs::msg::DisplayTrajectory disp;
    // get the current robot state to set the start state of the display trajectory
    moveit::core::robotStateToRobotStateMsg(*move_group_->getCurrentState(), disp.trajectory_start);
    // add the planned trajectory to the display message
    disp.trajectory.push_back(plan.trajectory_);
    display_pub_->publish(disp);
    // small delay to ensure the message is published and potentially rendered in rviz
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // wait for user confirmation via terminal before execution
    std::cout << "\nPlan successful. Execute trajectory? (y/N): ";
    std::string user_input;
    std::getline(std::cin, user_input); // read user input from console

    // check user input for confirmation ('y' or 'Y')
    if (user_input != "y" && user_input != "Y") {
      RCLCPP_WARN(get_logger(), "Execution aborted by user.");
      res->success = false;
      res->message = "Execution aborted by user.";
      return;
    }

    // execute the planned trajectory
    RCLCPP_INFO(get_logger(), "Executing trajectory...");
    moveit::core::MoveItErrorCode execute_result = move_group_->execute(plan.trajectory_);

    // check execution result
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

  // node members (interfaces, publishers, subscribers, services, state)
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_;

  rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>::SharedPtr display_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr waypoint_sub_;
  rclcpp::Service<Trigger>::SharedPtr exec_srv_;

  std::string planning_group_, waypoint_topic_, execute_service_, robot_base_frame_;
  int max_waypoints_;
  double floor_check_z_;
  std::vector<geometry_msgs::msg::Pose> latest_waypoints_; // stores the most recently received waypoints
};

int main(int argc, char **argv)
{
  // initialize ros 2 client library
  rclcpp::init(argc, argv);
  // create the node instance
  // prevent node from declaring parameters already set via launch files or command line
  auto node = std::make_shared<RobotTrajectoryExecutor>(
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(false)
  );
  // spin the node to process callbacks
  rclcpp::spin(node);
  // shutdown ros 2 client library
  rclcpp::shutdown();
  return 0;
}