import os
from ament_index_python.packages import get_package_share_directory
# core launch functionalities for defining and running launch files
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
# ros-specific launch actions, like launching nodes
from launch_ros.actions import Node
# used for conditional launching (e.g., only launch rviz if requested)
from launch.conditions import IfCondition


# OpaqueFunction allows accessing launch configuration values during execution
def launch_setup(context, *args, **kwargs):

    # retrieve the launch configuration values defined below
    # zed camera parameters
    camera_model = LaunchConfiguration('camera_model')
    node_name = LaunchConfiguration('node_name') # name for the zed node instance

    # universal robots parameters
    ur_type = LaunchConfiguration('ur_type')
    robot_ip = LaunchConfiguration('robot_ip')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    # use_sim_time = LaunchConfiguration('use_sim_time') # uncomment if using gazebo or other simulation

    # perception related parameters
    world_frame = LaunchConfiguration('world_frame')
    robot_base_frame = LaunchConfiguration('robot_base_frame')

    # static transform placeholder arguments (these need calibration)
    calib_odom_x = LaunchConfiguration('calib_odom_x')
    calib_odom_y = LaunchConfiguration('calib_odom_y')
    calib_odom_z = LaunchConfiguration('calib_odom_z')
    calib_odom_roll = LaunchConfiguration('calib_odom_roll')
    calib_odom_pitch = LaunchConfiguration('calib_odom_pitch')
    calib_odom_yaw = LaunchConfiguration('calib_odom_yaw')

    # rviz launch argument
    launch_rviz = LaunchConfiguration('launch_rviz')

    # determine the installation paths for required packages
    # use perform(context) to get the actual string value from LaunchConfiguration
    ur_type_str = ur_type.perform(context)
    world_frame_str = world_frame.perform(context)
    robot_base_frame_str = robot_base_frame.perform(context)

    # locate the zed_wrapper package
    try:
        pkg_zed_wrapper = get_package_share_directory('zed_wrapper')
    except Exception as e:
        print(f"error: 'zed_wrapper' package not found. installation might be missing or incomplete. {e}")
        # re-raising the exception provides a clearer failure point
        raise e

    # locate the ur driver and moveit config packages
    try:
        pkg_ur_moveit_config = get_package_share_directory('ur_moveit_config')
        pkg_ur_robot_driver = get_package_share_directory('ur_robot_driver')
    except Exception as e:
        print(f"error: ur packages ('ur_robot_driver' or '{ur_type_str}_moveit_config') not found. installation might be missing or incomplete. {e}")
        raise e

    # locate the custom robot_perception package
    try:
        pkg_robot_perception = get_package_share_directory('robot_perception')
    except Exception as e:
        print(f"error: 'robot_perception' package not found. ensure it's built and sourced correctly. {e}")
        raise e

    # configure and include the zed camera launch file
    zed_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            # construct the full path to the zed launch file
            PathJoinSubstitution([pkg_zed_wrapper, 'launch', 'zed_camera.launch.py'])
        ),
        # pass necessary arguments to the included zed launch file
        launch_arguments={
            'camera_model': camera_model,
            'node_name': node_name,
            'pos_tracking.pos_tracking_enabled': 'true', # enable positional tracking
            'pos_tracking.base_frame': robot_base_frame,
            'pos_tracking.map_frame': 'map', # standard map frame used by zed
            'pos_tracking.odometry_frame': world_frame, # define the zed odometry frame name (e.g., 'odom')
            'point_cloud.point_cloud_enabled': 'true', # enable point cloud publishing
            'object_detection.od_enabled': 'false', # disable the built-in object detection if not used
            'mapping.mapping_enabled': 'false' # disable mapping unless required for the application
        }.items()
    )

    # configure and include the ur robot driver launch file
    # this launches ur_control.launch.py, which starts key components like
    # RobotStatePublisher, JointStateBroadcaster, and hardware interface controllers
    ur_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            # construct the full path to the ur control launch file
            PathJoinSubstitution([pkg_ur_robot_driver, 'launch', 'ur_control.launch.py'])
        ),
        # pass necessary arguments to the included ur launch file
        launch_arguments={
            'ur_type': ur_type,
            'robot_ip': robot_ip,
            'use_fake_hardware': use_fake_hardware,
            'launch_rviz': 'false', # disable rviz in the ur launch, we handle it separately
            # 'use_sim_time': use_sim_time, # uncomment if using a simulated environment
        }.items()
    )

    # TO DO:
    # define the static transform publisher node (placeholder calibration)
    # !!! important !!! replace the default '0.0' values in the DeclareLaunchArgument section below
    # with the *actual* measured or calibrated transform between the world_frame (e.g., odom) and robot_base_frame.
    static_tf_pub_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_odom_to_base_link_publisher', # descriptive node name
        output='screen', # display node output in the console
        arguments=[
            # use perform(context) to get the string value from LaunchConfiguration at runtime
            calib_odom_x.perform(context),    # x offset (meters)
            calib_odom_y.perform(context),    # y offset (meters)
            calib_odom_z.perform(context),    # z offset (meters)
            calib_odom_yaw.perform(context),  # yaw rotation (radians)
            calib_odom_pitch.perform(context),# pitch rotation (radians)
            calib_odom_roll.perform(context), # roll rotation (radians)
            world_frame_str,     # parent frame id ('odom')
            robot_base_frame_str # child frame id ('base_link')
        ]
    )

    # define the node for the simple object snapshot functionality
    object_snapshot_node = Node(
        package='robot_perception',
        executable='object_snapshot', # should match the entry point defined in setup.py
        name='object_snapshot_node', # should match the node name used in the script
        output='screen', # display node output
        parameters=[{
            # construct the point cloud topic name using the evaluated zed node_name
            'point_cloud_topic': PathJoinSubstitution(['/zed/zed_node/point_cloud/cloud_registered']), # corrected topic path syntax
            'world_frame': world_frame, # pass the LaunchConfiguration directly
            'robot_base_frame': robot_base_frame, # pass the LaunchConfiguration directly
            # additional parameters can be set here directly or via more LaunchConfigurations
            # 'object_dims': [0.5, 0.3, 0.1], # example object dimensions
            # 'max_filter_distance': 0.75, # example filter setting
            # 'min_filter_distance': 0.1, # example filter setting
        }]
    )

    # define the rviz visualization node
    rviz_config_file = PathJoinSubstitution(
        # assumes the rviz configuration file is located in robot_perception/rviz directory
        [pkg_robot_perception, 'rviz', 'snapshot_config.rviz']
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file], # specify the rviz configuration file to load
        condition=IfCondition(launch_rviz) # launch this node only if the 'launch_rviz' argument is true
    )

    # collect all actions to be executed by the launch system
    # the order can be important for dependencies (e.g., ensure tf is published before nodes use it)
    actions_list = [
        zed_launch_include,    # start the zed camera driver and nodes
        ur_launch_include,     # start the ur robot driver and controllers
        static_tf_pub_node,    # publish the static transform (using placeholder values initially)
        object_snapshot_node,  # start the perception node which relies on transforms
        rviz_node              # start rviz visualization if requested
    ]

    # return the list of actions to the launch system
    return actions_list


def generate_launch_description():
    """generates the top-level launch description object."""
    # declare the launch arguments that can be set when launching this file
    declared_arguments = [
        # zed camera arguments
        DeclareLaunchArgument('camera_model', default_value='zedxm', description='specify the zed camera model (e.g., zed, zed2, zedm, zedx, zedxm)'),
        DeclareLaunchArgument('node_name', default_value='zed_node', description='specify the name for the zed node instance'),
        # ur robot arguments
        DeclareLaunchArgument('ur_type', default_value='ur10', description='specify the ur robot model (e.g., ur3, ur3e, ur5, ur5e, ur10, ur10e, ur16e, ur20, ur30)'),
        DeclareLaunchArgument('robot_ip', default_value='192.168.0.100', description='ip address of the ur robot controller (update if necessary)'), # update if known
        DeclareLaunchArgument('use_fake_hardware', default_value='true', description='set to true to use the fake hardware interface for testing without a physical robot'),
        # DeclareLaunchArgument('use_sim_time', default_value='false', description='set to true when using a simulation environment like gazebo'), # uncomment if using simulation
        # tf frame arguments
        DeclareLaunchArgument('world_frame', default_value='odom', description='frame id for the world/odometry reference (typically \'odom\')'),
        DeclareLaunchArgument('robot_base_frame', default_value='base_link', description='frame id for the robot\'s base (e.g., \'base_link\', \'base\', or \'tool0\' for ur)'),
        # placeholder tf values for the transform from world_frame to robot_base_frame
        # !!! important: replace these default '0.0' values with actual calibration results !!!
        DeclareLaunchArgument('calib_odom_x', default_value='0.0', description='[placeholder] calibrated x offset from world_frame to robot_base_frame (meters)'),
        DeclareLaunchArgument('calib_odom_y', default_value='0.0', description='[placeholder] calibrated y offset from world_frame to robot_base_frame (meters)'),
        DeclareLaunchArgument('calib_odom_z', default_value='0.0', description='[placeholder] calibrated z offset from world_frame to robot_base_frame (meters)'),
        DeclareLaunchArgument('calib_odom_roll', default_value='0.0', description='[placeholder] calibrated roll angle from world_frame to robot_base_frame (radians)'),
        DeclareLaunchArgument('calib_odom_pitch', default_value='0.0', description='[placeholder] calibrated pitch angle from world_frame to robot_base_frame (radians)'),
        DeclareLaunchArgument('calib_odom_yaw', default_value='0.0', description='[placeholder] calibrated yaw angle from world_frame to robot_base_frame (radians)'),
        # rviz argument
        DeclareLaunchArgument('launch_rviz', default_value='true', description='set to true to launch rviz2 with the specified configuration'),
    ]

    # create the final LaunchDescription object, including declared arguments and the OpaqueFunction
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
