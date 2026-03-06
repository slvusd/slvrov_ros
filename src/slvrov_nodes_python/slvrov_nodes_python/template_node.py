
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class TemplateNode(Node):
    def __init__(self):
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
