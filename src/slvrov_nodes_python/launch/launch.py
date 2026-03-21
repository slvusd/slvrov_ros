import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
 
 
def _cfg() -> str:
    return os.path.join(get_package_share_directory("slvrov_nodes_python"), "config")
 
 
def generate_launch_description() -> LaunchDescription:
 
    mappings_yaml_arg = DeclareLaunchArgument(
        "mappings_yaml",
        default_value=os.path.join(_cfg(), "joy_mappings.yaml"),
        description="Path to joy_mappings.yaml produced by multi_joy_calibrator",
    )
 
    cfg = os.path.join(_cfg(), "slvrov_config.yaml")
 
    joy_left = Node(
        package="joy",
        executable="joy_node",
        name="joy_node_left",
        parameters=[cfg],
        remappings=[("/joy", "/joy_left")],
        output="screen",
    )
 
    joy_right = Node(
        package="joy",
        executable="joy_node",
        name="joy_node_right",
        parameters=[cfg],
        remappings=[("/joy", "/joy_right")],
        output="screen",
    )
 
    joystick_logic = Node(
        package="slvrov_nodes_python",
        executable="joystick_logic_node",
        name="joystick_logic_node",
        parameters=[cfg, LaunchConfiguration("mappings_yaml")],
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
