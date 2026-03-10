
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class TemplateNode(Node):
    """Minimal example node used as a starting point for new ROS2 nodes."""

    def __init__(self):
        """Initialize the placeholder ROS2 node name."""
        # Minimal skeleton for new ROS2 Python nodes in this package.
        super().__init__("node_name")
        

def main(args=None):
    # Initialize and run node
    try:
        rclpy.init()
        node = TemplateNode()
        rclpy.spin(node)

    except (KeyboardInterrupt, ExternalShutdownException): 
        print("Shutdown signal received, exiting...")

    # Destroy node (now) and gracefully exit
    finally:
        if node is not None: node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__':
    main()
