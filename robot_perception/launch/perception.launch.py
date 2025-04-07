from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node, PushRosNamespace

def generate_launch_description():
    """generates the launch description for the robot perception system."""

    # declare launch arguments

    # arguments for zed wrapper node
    camera_model = LaunchConfiguration('camera_model', default='zedxm')
    node_name = LaunchConfiguration('node_name', default='zed_node') # name for the zed node instance

    # arguments for robot_perception nodes
    use_object_processor = LaunchConfiguration('use_object_processor', default='true')
    # arguments for object_processor_node parameters
    target_frame = LaunchConfiguration('target_frame', default='odom')
    confidence_threshold = LaunchConfiguration('confidence_threshold', default='0.6')
    collision_padding = LaunchConfiguration('collision_padding', default='0.05')
    marker_lifetime_s = LaunchConfiguration('marker_lifetime_s', default='1.0')

    # get package directories
    try:
        zed_wrapper_dir = get_package_share_directory('zed_wrapper')
    except Exception as e:
        print(f"error finding zed_wrapper package. please ensure it's installed and sourced: {e}")
        return None

    try:
        robot_perception_dir = get_package_share_directory('robot_perception')
    except Exception as e:
        print(f"error finding robot_perception package: {e}")
        return None

    # zed wrapper launch inclusion

    # path to the zed camera launch file provided by the zed-ros2-wrapper installation
    zed_launch_file_path = PathJoinSubstitution(
        [zed_wrapper_dir, 'launch', 'zed_camera.launch.py']
    )

    # include the zed wrapper launch file, passing necessary parameters
    zed_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(zed_launch_file_path),
        launch_arguments={
            # pass launchconfigurations to allow overriding from this launch file's command line
            'camera_model': camera_model,
            'node_name': node_name,
            # --- critical zed parameters ---
            'object_detection.od_enabled': 'true',        # ensure od is requested
            'object_detection.tracking_enabled': 'true',  # enable object tracking within the sdk
            'object_detection.markers_enabled': 'false',   # disable zed's internal markers, object_processor publishes its own
            'pos_tracking.pos_tracking_enabled': 'true',   # required for tf: odom -> base_link -> camera_link
            'mapping.mapping_enabled': 'false',           # disable spatial mapping unless specifically needed
            'general.sdk_verbose': '1',                   # set sdk verbosity level
        }.items() # convert dict to list of tuples for launch_arguments
    )

    # robot perception nodes

    # group perception nodes under a specific namespace for organization
    robot_perception_nodes_group = GroupAction(
        actions=[
            # push the namespace for the subsequent nodes in this group
            PushRosNamespace('robot_perception'),

            # node to process zed object detections into collisionobjects and markers
            Node(
                package='robot_perception',
                executable='object_processor',
                name='object_processor_node', # node name will be /robot_perception/object_processor_node
                parameters=[{
                    # subscribe to the correctly namespaced topic from the zed node
                    'object_detect_topic': PathJoinSubstitution(['/zed', node_name, 'obj_det/objects']),
                    # pass through parameters defined above
                    'target_frame': target_frame,
                    'confidence_threshold': confidence_threshold,
                    'collision_padding': collision_padding,
                    'marker_lifetime_s': marker_lifetime_s,
                    # default marker colors
                    'marker_color_r': 0.0,
                    'marker_color_g': 1.0,
                    'marker_color_b': 0.0,
                    'marker_color_a': 0.5,
                }],
                output='screen',
                # condition to only launch this node if use_object_processor is true
                condition=IfCondition(
                    PythonExpression(["'", use_object_processor, "' == 'true'"])
                )
            ),

        ]
    )

    # create launch description
    ld = LaunchDescription()

    # declare arguments that can be passed via command line (e.g, ros2 launch ... target_frame:=map)
    # these match the launchconfigurations defined above
    ld.add_action(DeclareLaunchArgument('camera_model', default_value='zedxm', description='zed camera model (e.g., zed, zed2, zed2i, zedm, zedx, zedxm)'))
    ld.add_action(DeclareLaunchArgument('node_name', default_value='zed_node', description='name for the zed node instance'))
    ld.add_action(DeclareLaunchArgument('use_object_processor', default_value='true', description='launch the object processor node'))
    ld.add_action(DeclareLaunchArgument('target_frame', default_value='odom', description='target frame id for collisionobjects and markers'))
    ld.add_action(DeclareLaunchArgument('confidence_threshold', default_value='0.6', description='minimum confidence for detected objects'))
    ld.add_action(DeclareLaunchArgument('collision_padding', default_value='0.05', description='padding added to collision object dimensions (meters)'))
    ld.add_action(DeclareLaunchArgument('marker_lifetime_s', default_value='1.0', description='lifetime (seconds) for visualization markers'))

    # add the actions to the launch description
    ld.add_action(zed_camera_launch)        # launch zed node first
    ld.add_action(robot_perception_nodes_group) # launch our perception nodes under their namespace

    return ld

