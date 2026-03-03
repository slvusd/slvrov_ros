
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from slvrov_tools.pca9685 import *
from slvrov_interfaces.msg import PCA9685Command

# PCA9685Command
# string[] id e.g. ["thruster_1", "thruster_2"]
# float32[] pwm percentage e.g. [0.5, -0.5]

from .submersed_globals import *


class PCA9685Node(Node):
    def __init__(self):

        super().__init__("pca9685_node")

        self.declare_parameter("frequency_hz", 50)
        self.frequency = self.get_parameter("frequency_hz").value

        self.pca9685 = PCA9685(I2C_BUS1, self.frequency)
        self.pin_configs = get_pca9685_pin_configs(PCA9685_PIN_CONFIG_PATH)
        self.get_logger().info(f"Retrieved PCA9685 pin configs: {self.pin_configs}")

        self.pca9685_command_subscription = self.create_subscription(PCA9685Command, "pca9685_command", self.pca9685_command_callback, 10)

    def pca9685_command_callback(self, msg):
        try:
            for id_, pwm in zip(list(msg.id), list(msg.pwm)):

                # Don't want to raise Exception b/c it will crash node and we want to be able to continue receiving commands, but log error for debugging
                if id_ not in self.pin_configs: self.get_logger().error(f"Invalid PCA9685 pin id: {id_}, skipping command")
                else:
                    pin_config = self.pin_configs[id_]

                    if pwm >= 0: duty_cycle = int(pin_config["default"] + pwm * (pin_config["maximum"] - pin_config["default"]))
                    else: duty_cycle = int(pin_config["default"] + pwm * (pin_config["minimum"] - pin_config["default"]))

                    for pin in pin_config["pins"]:
                        self.pca9685.write_duty_cycle(pin, duty_cycle)
                        self.get_logger().info(f"Setting pin {pin} to duty cycle {duty_cycle}")

        except Exception as exception:
            self.get_logger().error(f"Pins: {msg.id}, PWMs: {msg.pwm}\nERROR: {exception}")
        

def main(args=None):
    # Initialize and run node
    try:
        rclpy.init()
        node = PCA9685Node()
        rclpy.spin(node)

    except (KeyboardInterrupt, ExternalShutdownException): 
        print("Shutdown signal received, exiting...")

    # Destroy node (now) and gracefully exit
    finally:
        if node is not None: node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__':
    main()
