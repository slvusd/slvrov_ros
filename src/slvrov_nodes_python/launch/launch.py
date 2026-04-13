import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

from slvrov_tools.joystick_tools import get_available_joysticks

# TODO
# Args:
# joystick node names -- defaults to js$index
# joystick config path
# pwm topic destination

# in other launch...
# pin config path
 
 
def _cfg() -> str:
    return os.path.join(get_package_share_directory("slvrov_nodes_python"), "config")
 
 
def generate_launch_description() -> LaunchDescription:
 
    mappings_yaml_arg = DeclareLaunchArgument(
        "mappings_yaml",
        default_value=os.path.join(_cfg(), "joy_mappings.yaml"),
        description="Path to joy_mappings.yaml produced by multi_joy_calibrator",
    )
 
    cfg = os.path.join(_cfg(), "slvrov_config.yaml")

    joystick_indecies = get_available_joysticks()
    joystick_nodes = [Node(package="joy", executable="joy_node", name=f"js{index}_node", parameters=[cfg], remappings=[("/joy", f"/js{index}")], output="screen",) for index in joystick_indecies]
 
 # TODO: match mapping param set to cli arg?
    joystick_logic = Node(
        package="slvrov_nodes_python",
        executable="joystick_logic_node",
        name="joystick_logic_node",
        parameters=["mappings_file", LaunchConfiguration("mappings_yaml")],
        output="screen",
    )
 
    #thruster_bridge = Node(
    #   package="slvrov_nodes_python",
    #    executable="thruster_bridge",
    #    name="thruster_bridge_node",
    #    output="screen",
    #)
 
    return LaunchDescription([
        mappings_yaml_arg,
        joy_left,
        joy_right,
        joystick_logic,
        thruster_bridge,
    ])
