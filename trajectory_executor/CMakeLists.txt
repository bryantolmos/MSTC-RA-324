cmake_minimum_required(VERSION 3.8)
project(trajectory_executor)

# C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(pilz_industrial_motion_planner REQUIRED)

add_executable(robot_trajectory_executor src/robot_trajectory_executor.cpp)
ament_target_dependencies(robot_trajectory_executor
  rclcpp
  moveit_ros_planning_interface
  moveit_core
  pilz_industrial_motion_planner
  moveit_msgs
  geometry_msgs
  std_srvs
)

install(
  TARGETS robot_trajectory_executor
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.py"
)

ament_package()
