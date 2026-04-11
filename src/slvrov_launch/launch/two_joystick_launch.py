from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "config_file",
            default_value="two_joystick_launch.yaml",
            description="YAML file name inside slvrov_launch/config",
        ),

        DeclareLaunchArgument(
            "js0_topic", 
            default_value="/godzillah/js0",
            description=""),

        DeclareLaunchArgument(
            "js1_topic", 
            default_value="/godzillah/js1",
            description=""),
    ]

    config_path = PathJoinSubstitution([
        FindPackageShare("slvrov_launch"),
        "config",
        LaunchConfiguration("config_file"),
    ])

    nodes = [
        Node(
            package="joy",
            executable="joy_node",
            namespace="godzillah",
            name="js0",
            parameters=[config_path],
            remappings=[("joy", LaunchConfiguration("js0_topic"))],
            output="screen",
        ),

        Node(
            package="joy",
            executable="joy_node",
            namespace="godzillah",
            name="js1",
            parameters=[config_path],
            remappings=[("joy", LaunchConfiguration("js1_topic"))],
            output="screen",
        ),

        Node(
            package="slvrov_nodes_python",
            executable="joystick_logic",
            namespace="godzillah",
            name="js_logic",
            parameters=[
                config_path,
            ],
            output="screen",
        ),
    ]

    return LaunchDescription(declared_arguments + nodes)