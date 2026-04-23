import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Override with SLVROV_REPO env var or the mapping_file launch argument.
_REPO = os.environ.get("SLVROV_REPO", "/home/pi/slvrov_ros")


def generate_launch_description() -> LaunchDescription:

    mapping_arg = DeclareLaunchArgument(
        "mapping_file",
        default_value=os.path.join(_REPO, "joy_mapping.yaml"),
        description="Absolute path to joy_mapping.yaml",
    )

    return LaunchDescription([
        mapping_arg,

        Node(
            package="slvrov_nodes_python",
            executable="pca9685_pin_configs_server",
            name="pca9685_pin_configs_server",
            output="screen",
        ),

        Node(
            package="slvrov_nodes_python",
            executable="joystick_logic",
            name="joystick_logic_node",
            parameters=[{"mapping_file": LaunchConfiguration("mapping_file")}],
            output="screen",
        ),

        Node(
            package="slvrov_nodes_python",
            executable="thruster_bridge",
            name="thruster_bridge_node",
            output="screen",
        ),
    ])
