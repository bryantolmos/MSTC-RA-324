from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    """Launch robot_trajectory_executor with configurable parameters."""

    # declare launch arguments that can be overridden
    declared_args = [
        DeclareLaunchArgument(
            'planning_group', default_value='ur_manipulator',
            description='moveit planning group name for trajectory execution'
        ),
        DeclareLaunchArgument(
            'robot_base_frame', default_value='base_link',
            description='base frame id of the robot (must match waypoint frame)'
        ),
        DeclareLaunchArgument(
            'floor_check_z', default_value='-0.05',
            description='safety z threshold below which waypoints are rejected'
        ),
        DeclareLaunchArgument(
            'waypoint_offset_z', default_value='0.05',
            description='offset to apply to the Z coordinate of received waypoints (in base frame)'
        ),
        # optional: uncomment below to expose more parameters as launch arguments
        # DeclareLaunchArgument('waypoint_topic', default_value='/planned_waypoints', description='topic for receiving PoseArray waypoints'),
        # DeclareLaunchArgument('execute_service_name', default_value='/execute_web_trajectory', description='service name to trigger trajectory execution'),
        # DeclareLaunchArgument('cartesian_step_size', default_value='0.01', description='step size for cartesian path planning'),
        # DeclareLaunchArgument('cartesian_jump_threshold', default_value='0.0', description='jump threshold for cartesian path planning (0.0 disables check)'),
        # DeclareLaunchArgument('min_plan_fraction', default_value='0.9', description='minimum fraction of waypoints required for a valid plan'),
        # DeclareLaunchArgument('max_waypoints', default_value='50', description='maximum number of waypoints allowed in a trajectory'),
    ]

    # map launch configurations to node parameters
    params = {
        'planning_group': LaunchConfiguration('planning_group'),
        'robot_base_frame': LaunchConfiguration('robot_base_frame'),
        'floor_check_z': LaunchConfiguration('floor_check_z'),
        'waypoint_offset_z': LaunchConfiguration('waypoint_offset_z'),
        # if uncommenting arguments above, add corresponding launch configurations here:
        # 'waypoint_topic': LaunchConfiguration('waypoint_topic'),
        # 'execute_service_name': LaunchConfiguration('execute_service_name'),
        # 'cartesian_step_size': LaunchConfiguration('cartesian_step_size'),
        # 'cartesian_jump_threshold': LaunchConfiguration('cartesian_jump_threshold'),
        # 'min_plan_fraction': LaunchConfiguration('min_plan_fraction'),
        # 'max_waypoints': LaunchConfiguration('max_waypoints'),
    }

    # define the node execution
    executor_node = Node(
        package='trajectory_executor',
        executable='robot_trajectory_executor',
        name='robot_trajectory_executor',
        output='screen',
        parameters=[params], # pass the dictionary of parameters
        # optional: automatically declares parameters from the dictionary if not already declared in the node
        # automatically_declare_parameters_from_overrides=True,
    )

    # return the complete launch description
    return LaunchDescription(declared_args + [executor_node])