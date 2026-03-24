import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    mappings_yaml_arg = DeclareLaunchArgument(
        "mappings_yaml",
        default_value=os.path.join(os.getcwd(), "joy_mappings.yaml"),
        description="Path to joy_mappings.yaml produced by joystick_calibrator",
    )

    joy_left = Node(
        package="joy",
        executable="joy_node",
        name="joy_node_left",
        parameters=[{"device_id": 0}],
        remappings=[("/joy", "/joy_left")],
        output="screen",
    )

    joy_right = Node(
        package="joy",
        executable="joy_node",
        name="joy_node_right",
        parameters=[{"device_id": 1}],
        remappings=[("/joy", "/joy_right")],
        output="screen",
    )

    joystick_logic = Node(
        package="slvrov_nodes_python",
        executable="joystick_logic",
        name="joystick_logic_node",
        parameters=[{"mapping_file": LaunchConfiguration("mappings_yaml")}],
        output="screen",
    )

    thruster_bridge = Node(
        package="slvrov_nodes_python",
        executable="thruster_bridge",
        name="thruster_bridge_node",
        output="screen",
    )

    return LaunchDescription([
        mappings_yaml_arg,
        joy_left,
        joy_right,
        joystick_logic,
        thruster_bridge,
    ])
