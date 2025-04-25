from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction # opaque function allows for evaluating launch args before node creation
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import sys

def launch_setup(context, *args, **kwargs):
    """
    Evaluates launch arguments and creates Node instances with correct parameter types.

    This function is executed by OpaqueFunction after launch arguments are parsed.
    It converts string representations of numbers/lists/booleans into their
    actual types before passing them to the Node definitions.
    """
    # retrieve the string values of launch configurations using context
    parent_frame_str = LaunchConfiguration('parent_frame').perform(context)
    object_frame_str = LaunchConfiguration('object_frame').perform(context)
    pos_x_str = LaunchConfiguration('pos_x').perform(context)
    pos_y_str = LaunchConfiguration('pos_y').perform(context)
    pos_z_str = LaunchConfiguration('pos_z').perform(context)
    ori_x_str = LaunchConfiguration('ori_x').perform(context)
    ori_y_str = LaunchConfiguration('ori_y').perform(context)
    ori_z_str = LaunchConfiguration('ori_z').perform(context)
    ori_w_str = LaunchConfiguration('ori_w').perform(context)
    dim_l_str = LaunchConfiguration('dim_l').perform(context)
    dim_h_str = LaunchConfiguration('dim_h').perform(context)
    dim_w_str = LaunchConfiguration('dim_w').perform(context)
    floor_enable_str = LaunchConfiguration('floor_enable').perform(context)
    floor_pos_z_str = LaunchConfiguration('floor_pos_z').perform(context)
    floor_size_x_str = LaunchConfiguration('floor_size_x').perform(context)
    floor_size_y_str = LaunchConfiguration('floor_size_y').perform(context)
    floor_size_z_str = LaunchConfiguration('floor_size_z').perform(context)

    # convert string arguments to their appropriate types (float, list, bool)
    try:
        pose_position_x = float(pos_x_str)
        pose_position_y = float(pos_y_str)
        pose_position_z = float(pos_z_str)
        pose_orientation_x = float(ori_x_str)
        pose_orientation_y = float(ori_y_str)
        pose_orientation_z = float(ori_z_str)
        pose_orientation_w = float(ori_w_str)
        # create lists for dimensions and size
        dimensions_list = [float(dim_l_str), float(dim_h_str), float(dim_w_str)]
        floor_enable_bool = floor_enable_str.lower() in ['true', '1'] # handle boolean conversion robustly
        floor_pose_z = float(floor_pos_z_str)
        floor_size_list = [float(floor_size_x_str), float(floor_size_y_str), float(floor_size_z_str)]
    except ValueError as e:
        # log error clearly if conversion fails
        print(f"\n\nERROR converting launch arguments to float/bool: {e}", file=sys.stderr)
        # returning None or raising an exception would stop the launch
        return None # or raise e

    # create node instances using the evaluated and converted parameters
    object_publisher_node = Node(
        package='known_object_handler',
        executable='object_publisher_node',
        name='object_publisher_node',
        output='screen',
        parameters=[{ # parameters are passed as a dictionary to the node
            'parent_frame': parent_frame_str,
            'object_frame': object_frame_str,
            'pose.position.x': pose_position_x,
            'pose.position.y': pose_position_y,
            'pose.position.z': pose_position_z,
            'pose.orientation.x': pose_orientation_x,
            'pose.orientation.y': pose_orientation_y,
            'pose.orientation.z': pose_orientation_z,
            'pose.orientation.w': pose_orientation_w,
            'dimensions': dimensions_list, # pass the converted list for dimensions
            'floor.enable': floor_enable_bool,
            'floor.pose.position.z': floor_pose_z,
            'floor.size': floor_size_list # pass the converted list for floor size
        }]
    )

    waypoint_manager_node = Node(
        package='known_object_handler',
        executable='waypoint_manager_node',
        name='waypoint_manager_node',
        output='screen',
        parameters=[{ # pass parameters as a dictionary
            # waypoint manager needs the frames to perform transformations
            'robot_base_frame': parent_frame_str, # use the evaluated parent_frame
            'known_object_frame': object_frame_str, # use the evaluated object_frame
        }]
    )

    # return the list of nodes to be launched
    return [object_publisher_node, waypoint_manager_node]


def generate_launch_description():
    """Launches the Python nodes for known object handling and waypoint management."""

    # declare launch arguments with default values
    # these arguments can be overridden from the command line or other launch files
    declared_args = [
        DeclareLaunchArgument('parent_frame', default_value='base_link', description='Base frame for the object pose'),
        DeclareLaunchArgument('object_frame', default_value='known_object_frame', description='TF frame name for the object'),
        # object pose relative to parent_frame
        DeclareLaunchArgument('pos_x', default_value='0.0', description='Object position X'),
        DeclareLaunchArgument('pos_y', default_value='0.55', description='Object position Y'),
        DeclareLaunchArgument('pos_z', default_value='0.15', description='Object position Z'),
        DeclareLaunchArgument('ori_x', default_value='0.0', description='Object orientation X (quaternion)'),
        DeclareLaunchArgument('ori_y', default_value='0.0', description='Object orientation Y (quaternion)'),
        DeclareLaunchArgument('ori_z', default_value='0.0', description='Object orientation Z (quaternion)'),
        DeclareLaunchArgument('ori_w', default_value='1.0', description='Object orientation W (quaternion)'),
        # object dimensions (L, H, W)
        DeclareLaunchArgument('dim_l', default_value='0.5', description='Object dimension Length (X)'),
        DeclareLaunchArgument('dim_h', default_value='0.1', description='Object dimension Height (Y)'), # Note: Check if this should be Y or Z based on node interpretation
        DeclareLaunchArgument('dim_w', default_value='0.36', description='Object dimension Width (Z)'), # Note: Check if this should be Z or Y based on node interpretation
        # floor parameters
        DeclareLaunchArgument('floor_enable', default_value='true', description='Enable floor collision object'),
        DeclareLaunchArgument('floor_pos_z', default_value='-0.02', description='Floor Z position relative to parent_frame'),
        DeclareLaunchArgument('floor_size_x', default_value='2.0', description='Floor size X'),
        DeclareLaunchArgument('floor_size_y', default_value='2.0', description='Floor size Y'),
        DeclareLaunchArgument('floor_size_z', default_value='0.01', description='Floor thickness Z'),
    ]

    # use OpaqueFunction to delay node creation until launch arguments are evaluated
    # this allows passing correctly typed parameters (floats, lists, bools) to the nodes
    return LaunchDescription(declared_args + [OpaqueFunction(function=launch_setup)])