#!/bin/bash

# --- Configuration ---
WS_PATH="$HOME/ws_jesus"
ROS_SETUP="/opt/ros/humble/setup.bash"
WS_SETUP="$WS_PATH/install/setup.bash"

# UR Configuration
UR_TYPE="ur10"
# Set to true for simulation/testing, false for real robot
USE_FAKE_HARDWARE="true"
# Set the correct IP if using the real robot
ROBOT_IP="192.168.1.102"

# --- Helper Function ---
# Sources ROS and workspace, then executes command in a new terminal
run_in_terminal() {
  local description=$1
  local command_to_run=$2
  echo "Launching: $description"
  # Using gnome-terminal: starts a new terminal, sources ROS, runs command, keeps terminal open
  gnome-terminal -- bash -c "source $ROS_SETUP; source $WS_SETUP; echo '>>> Running: $description'; echo '>>> Command: $command_to_run'; echo; $command_to_run; echo '>>> Command finished. Press Ctrl+C to close this terminal.'; exec bash" &
  # exec bash keeps the terminal open after the command finishes
}

# --- Launch Sequence ---

echo "--- Starting Robot Perception and Control System ---"

# 1. Launch UR Driver
UR_DRIVER_DESC="UR Driver ($UR_TYPE, Fake HW: $USE_FAKE_HARDWARE)"
UR_DRIVER_CMD="ros2 launch ur_robot_driver ur_control.launch.py ur_type:=$UR_TYPE use_fake_hardware:=$USE_FAKE_HARDWARE launch_rviz:=false robot_ip:=$ROBOT_IP"
run_in_terminal "$UR_DRIVER_DESC" "$UR_DRIVER_CMD"
echo "Waiting for UR Driver to initialize (8s)..."
sleep 8

# 2. Launch MoveIt Config (includes move_group and RViz)
# RViz is launched here according to your original script snippet
MOVEIT_DESC="MoveIt ($UR_TYPE, With RViz)"
MOVEIT_CMD="ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=$UR_TYPE use_fake_hardware:=$USE_FAKE_HARDWARE launch_rviz:=true"
run_in_terminal "$MOVEIT_DESC" "$MOVEIT_CMD"
echo "Waiting for MoveIt to initialize (10s)..."
sleep 10 # Give MoveIt more time

# 3. Launch rosbridge WebSocket Server
ROSBRIDGE_DESC="rosbridge WebSocket"
ROSBRIDGE_CMD="ros2 run rosbridge_server rosbridge_websocket"
run_in_terminal "$ROSBRIDGE_DESC" "$ROSBRIDGE_CMD"
echo "Waiting for rosbridge (2s)..."
sleep 2

# 4. Launch Known Object Handler (Python Nodes + TF Publisher)
# This uses the default parameters defined in the launch file,
# Add arguments here (e.g., pos_x:=0.6) to override defaults.

# (Jesus) Override the same dimensions so the cube you’re clicking 
# on in the browser is the same shape as the collision object in RViz
OBJECT_HANDLER_DESC="Known Object Handler Nodes"
OBJECT_HANDLER_CMD="ros2 launch known_object_handler publish_known_object.launch.py \
  pos_x:=0.0 \
  pos_y:=0.7 \
  pos_z:=0.15 \
  dim_l:=0.5 dim_h:=0.1 dim_w:=0.3 \
  floor_enable:=true"
run_in_terminal "$OBJECT_HANDLER_DESC" "$OBJECT_HANDLER_CMD"
echo "Waiting for Object Handler (3s)..."
sleep 3

# 5. Launch Trajectory Executor (C++ Node)
# This uses the default parameters defined in its launch file.
EXECUTOR_DESC="Trajectory Executor Node"
EXECUTOR_CMD="ros2 launch trajectory_executor trajectory_executor.launch.py"
run_in_terminal "$EXECUTOR_DESC" "$EXECUTOR_CMD"
echo "Waiting for Trajectory Executor (2s)..."
sleep 2

echo "--- All ROS 2 components launched. ---"
echo "You can now start the Electron UI application."

# Optional: Add command to start Electron app automatically
# echo "Starting Electron App..."
# cd /path/to/your/electron/app && npm start &

exit 0
