#!/bin/bash

# --- Configuration ---
WS_PATH="$HOME/ws"
ROS_SETUP="/opt/ros/humble/setup.bash"
WS_SETUP="$WS_PATH/install/setup.bash"

# UR Configuration
UR_TYPE="ur10"
ROBOT_IP="192.168.1.102"

# --- Helper Function ---
run_in_terminal() {
  local description=$1
  local command_to_run=$2
  echo "Launching: $description"
  gnome-terminal -- bash -c "source $ROS_SETUP; source $WS_SETUP; echo '>>> Running: $description'; echo '>>> Command: $command_to_run'; echo; $command_to_run; echo '>>> Command finished. Press Ctrl+C to close this terminal.'; exec bash" &
}

# --- Launch Sequence ---

echo "--- Starting Robot Perception and Control System (Gazebo Sim) ---"

# 1. Launch Gazebo with UR robot
GAZEBO_DESC="Gazebo Simulation ($UR_TYPE)"
GAZEBO_CMD="ros2 launch ur_simulation_gz ur_sim_control.launch.py ur_type:=$UR_TYPE launch_rviz:=false use_fake_hardware:=true"
run_in_terminal "$GAZEBO_DESC" "$GAZEBO_CMD"
echo "Waiting for Gazebo to initialize (10s)..."
sleep 10

# 2. Launch MoveIt with simulation support
MOVEIT_DESC="MoveIt ($UR_TYPE, Gazebo, With RViz)"
MOVEIT_CMD="ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=$UR_TYPE use_fake_hardware:=true launch_rviz:=true use_sim_time:=true"
run_in_terminal "$MOVEIT_DESC" "$MOVEIT_CMD"
echo "Waiting for MoveIt to initialize (10s)..."
sleep 10

# 3. Launch rosbridge WebSocket Server
ROSBRIDGE_DESC="rosbridge WebSocket"
ROSBRIDGE_CMD="ros2 run rosbridge_server rosbridge_websocket"
run_in_terminal "$ROSBRIDGE_DESC" "$ROSBRIDGE_CMD"
echo "Waiting for rosbridge (2s)..."
sleep 2

# 4. Launch Known Object Handler
OBJECT_HANDLER_DESC="Known Object Handler Nodes"
OBJECT_HANDLER_CMD="ros2 launch known_object_handler publish_known_object.launch.py"
run_in_terminal "$OBJECT_HANDLER_DESC" "$OBJECT_HANDLER_CMD"
echo "Waiting for Object Handler (3s)..."
sleep 3

# 5. Launch Trajectory Executor
EXECUTOR_DESC="Trajectory Executor Node"
EXECUTOR_CMD="ros2 launch trajectory_executor trajectory_executor.launch.py"
run_in_terminal "$EXECUTOR_DESC" "$EXECUTOR_CMD"
echo "Waiting for Trajectory Executor (2s)..."
sleep 2

echo "--- All ROS 2 components launched in Gazebo Sim mode. ---"
echo "You can now start the Electron UI application."

exit 0

