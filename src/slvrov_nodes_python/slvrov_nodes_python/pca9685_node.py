
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
    """Translate logical PWM commands into PCA9685 duty-cycle writes."""

    def __init__(self):
        """Initialize the PCA9685 driver, config map, and command subscription."""

        super().__init__("pca9685_node")

        # Keep the hardware update rate configurable so the same node can drive
        # servos or ESCs that expect different PWM frequencies.
        self.declare_parameter("frequency_hz", 50)
        self.frequency = self.get_parameter("frequency_hz").value

        self.pca9685 = PCA9685(I2C_BUS1, self.frequency)
        # Load the persisted logical-id to pin mapping once at startup; command
        # callbacks rely on this lookup to translate topic messages into writes.
        self.pin_configs = get_pca9685_pin_configs(PCA9685_PIN_CONFIG_PATH)
        self.get_logger().info(f"Retrieved PCA9685 pin configs: {self.pin_configs}")

        self.pca9685_command_subscription = self.create_subscription(PCA9685Command, "pca9685_command", self.pca9685_command_callback, 10)

    def pca9685_command_callback(self, msg):
        """Apply commanded PWM values to the configured PCA9685 channel set.

        Args:
            msg: Incoming command message containing logical output IDs and PWM
                percentages.
        """
        try:
            for id_, pwm in zip(list(msg.id), list(msg.pwm)):

                # Invalid IDs are treated as bad input rather than fatal errors so
                # the node can continue servicing the rest of the batch.
                if id_ not in self.pin_configs: self.get_logger().error(f"Invalid PCA9685 pin id: {id_}, skipping command")
                else:
                    pin_config = self.pin_configs[id_]

                    # Positive and negative PWM values scale away from the neutral
                    # duty cycle toward the configured extrema for that device.
                    if pwm >= 0: duty_cycle = int(pin_config["default"] + pwm * (pin_config["maximum"] - pin_config["default"]))
                    else: duty_cycle = int(pin_config["default"] + pwm * (pin_config["minimum"] - pin_config["default"]))

                    # Some logical outputs fan out to multiple PCA9685 pins, so
                    # each configured channel receives the same duty cycle.
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
