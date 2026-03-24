#!/usr/bin/env python3
"""
thruster_bridge_node.py
=======================
Bridges joystick_logic_node output to pca9685_node input.
 
Subscribes to /thruster_command (published by joystick_logic_node) and
republishes every message unchanged onto /pca9685_command (consumed by
pca9685_node).
 
Keeping this as a separate node means:
  - multi_joy_logic.py has no knowledge of pca9685_node or pin configs
  - pca9685_node has no knowledge of the joystick pipeline
  - this file is the only thing that changes if the topic names or routing
    ever need to change
 
Subscriptions
─────────────
  /thruster_command   (slvrov_interfaces/msg/PCA9685Command)
      Published by joystick_logic_node with normalised [-1, 1] floats
      and logical IDs (thruster_1..N).
 
Publications
────────────
  /pca9685_command    (slvrov_interfaces/msg/PCA9685Command)
      Consumed by pca9685_node, which resolves IDs against the JSON pin
      configs and writes duty cycles to the PCA9685 chip over I²C.
"""
 
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
 
from slvrov_interfaces.msg import PCA9685Command
 
 
class ThrusterBridgeNode(Node):
    """Forward /thruster_command messages to /pca9685_command unchanged."""
 
    def __init__(self) -> None:
        super().__init__("thruster_bridge_node")
 
        self.pub = self.create_publisher(
            PCA9685Command, "pca9685_command", 10
        )
 
        self.create_subscription(
            PCA9685Command,
            "thruster_command",
            self._forward,
            10,
        )
 
        self.get_logger().info(
            "thruster_bridge_node ready: "
            "/thruster_command → /pca9685_command"
        )
 
    def _forward(self, msg: PCA9685Command) -> None:
        """Re-publish the incoming message on /pca9685_command.
 
        Args:
            msg: PCA9685Command from joystick_logic_node.
        """
        self.pub.publish(msg)
 
 
# ── Entry point ───────────────────────────────────────────────────────────────
 
def main(args=None) -> None:
    try:
        rclpy.init(args=args)
        node = ThrusterBridgeNode()
        rclpy.spin(node)
 
    except (KeyboardInterrupt, ExternalShutdownException):
        print("Shutdown signal received, exiting...")
 
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()
