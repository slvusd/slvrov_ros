from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def get_configuration() -> DeclareLaunchArgument:

    # User can define parameters and settings for the logic node
    # This is best done through the YAML, but for fast testing or temporary settings they can do it this way too
    config_file_arg = DeclareLaunchArgument(
        "config_file",
        default_value="test_launch.yaml",
        description="YAML file name inside slvrov_launch/rov_config/rovs",
    )

    return config_file_arg

def generate_launch_description():
    """Launch multiple nodes"""

    config_file_arg = get_configuration()
    config_file = LaunchConfiguration("config_file")

    # Build the full path to the config file at runtime
    config_path = PathJoinSubstitution([
        FindPackageShare('slvrov_launch'),
        'rov_config',
        'rovs',
        config_file,
    ])

    # Create a launch description
    ld = LaunchDescription()
    nodes = []

    # Publisher 1 node
    nodes.append(Node(
        package='demo_nodes_py',
        executable='talker',
        namespace='talkie',
        name='publisher_1',
        parameters=[config_path]
    ))

    # Publisher 2 node
    nodes.append(Node(
        package='demo_nodes_py',
        executable='talker',
        namespace='talkie',
        name='publisher_2',
        parameters=[config_path]
    ))

    # Subscriber node
    nodes.append(Node(
        package='demo_nodes_py',
        executable='listener',
        namespace='talkie',
        name='subscriber_1'
    ))

    # Add argument(s) to launch description
    ld.add_action(config_file_arg)

    # Add nodes to launch description
    for node in nodes:
        ld.add_action(node)

    return ld
