from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        Node(
            package="joy",
            executable="joy_node",
            name="joy_node_left",
            parameters=[{"device_id": 0, "autorepeat_rate": 20.0}],
            remappings=[("/joy", "/joy_left")],
            output="screen",
        ),

        Node(
            package="joy",
            executable="joy_node",
            name="joy_node_right",
            parameters=[{"device_id": 1, "autorepeat_rate": 20.0}],
            remappings=[("/joy", "/joy_right")],
            output="screen",
        ),
    ])
