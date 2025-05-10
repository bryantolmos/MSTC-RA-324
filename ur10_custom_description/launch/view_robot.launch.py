from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution, TextSubstitution


from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    description_package = FindPackageShare("ur10_custom_description")
    description_file = PathJoinSubstitution(
        [description_package, "urdf", "ur10_custom.urdf.xacro"]
    )
    rvizconfig_file = PathJoinSubstitution([description_package, "rviz", "urdf.rviz"])

    robot_description = ParameterValue(
    Command([
        "xacro ", description_file,
        " ", "name:=", "ur",
        " ", "ur_type:=", "ur10",
        " ", "sim_gazebo:=", "false",
        " ", "use_fake_hardware:=", "true",
        " ", "fake_sensor_commands:=", "false"
    ]),
    value_type=str)

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rvizconfig_file],
    )

    return LaunchDescription(
        [joint_state_publisher_gui_node, robot_state_publisher_node, rviz_node]
    )
