
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.example_interfaces.srv import SetString
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

        self.pca9685 = None
        self.connect_to_pca9685()

        # Load the persisted logical-id to pin mapping once at startup; command
        # callbacks rely on this lookup to translate topic messages into writes.
        self.pin_configs = None
        self.retrieve_pin_configs()

        self.create_service(SetString, "pca9685_node_actions", self.node_actions_callback)
        self.pca9685_command_subscription = self.create_subscription(PCA9685Command, "pca9685_command", self.pca9685_command_callback, 10)

    def retrieve_pin_configs(self):
        """Load the persisted PCA9685 pin configuration mapping from disk."""

        try:
            self.pin_configs = get_pca9685_pin_configs(PCA9685_PIN_CONFIG_PATH)
            self.get_logger().info(f"Retrieved PCA9685 pin configs: {self.pin_configs}")

        except FileNotFoundError: self.get_logger().warning(f"Pin configs file {PCA9685_PIN_CONFIG_PATH} not found.")
        except Exception as exception: self.get_logger().error(f"Error retrieving pin configs: {exception}")

    def connect_to_pca9685(self):
        """Attempt to connect to the PCA9685 and log any connection errors."""

        try:
            self.pca9685 = PCA9685(I2C_BUS1, self.frequency)
            self.get_logger().info("Successfully connected to PCA9685.")

        except OSError: self.get_logger().error("PCA9685 not found on I2C bus. Check your connections and try again.")
        except Exception as exception: self.get_logger().error(f"Failed to connect to PCA9685: {exception}")

    def node_actions_callback(self, req, resp):
        """Handle incoming service requests to perform node actions like refreshing pin configs and connecting to PCA9685.

        Args:
            req: Service request containing the action command. Valid commands include "retrieve_pin_configs" and "connect_to_pca9685".
            resp: Service response object to populate with the operation result.

        Returns:
            The populated service response.
        """
        try:
            command = req.data

            if command == "retrieve_pin_configs":
                self.get_logger().info("Received signal to refresh pin configs.")
                self.retrieve_pin_configs()

                if self.pin_configs is not None:
                    resp.success = True
                    resp.msg = "Pin configs refreshed successfully."
                else:
                    resp.success = False
                    resp.msg = "Failed to refresh pin configs. Check logs for details."

            elif command == "connect_to_pca9685":
                self.get_logger().info("Received signal to connect to PCA9685.")
                self.connect_to_pca9685()

                if self.pca9685 is not None:
                    resp.success = True
                    resp.msg = "Connected to PCA9685 successfully."
                else:
                    resp.success = False
                    resp.msg = "Failed to connect to PCA9685. Check logs for details."

            else:
                self.get_logger().warning(f"Received unknown command: {req.data}")

                resp.success = False
                resp.msg = f"Unknown command: {req.data}"

        except Exception as exception:
            resp.success = False
            resp.msg = f"An error occurred while processing the command: {exception}"
            
            self.get_logger().error(f"Command: {req.data}\nERROR: {exception}")

        return resp

    def pca9685_command_callback(self, msg):
        """Apply commanded PWM values to the configured PCA9685 channel set.

        Args:
            msg: Incoming command message containing logical output IDs and PWM
                percentages.
        """
        self.get_logger().info(f"Recieved message: Pins: {msg.id}, PWMs: {msg.pwm}")

        if self.pca9685 is None:
            self.get_logger().error("PCA9685 not initialized, cannot process command. Check connections and try again.")
            return

        if self.pin_configs is None:
            self.get_logger().error("No pin configs available, cannot process command. Use the pin config client to add configs and try again.")
            return

        try:
            for id_, pwm in zip(list(msg.id), list(msg.pwm)):

                # Invalid IDs are treated as bad input rather than fatal errors so
                # the node can continue servicing the rest of the batch.
                if id_ not in self.pin_configs: self.get_logger().error(f"Invalid PCA9685 pin id: {id_}, skipping command")
                else:
                    pin_config = self.pin_configs[id_]

                    # Positive and negative PWM values scale away from the neutral/default/middle duty cycle toward the configured extrema for that device.
                    if pwm >= 0: diff = pin_config["maximum"] - pin_config["default"]
                    else: diff = pin_config["default"] - pin_config["minimum"]
                    
                    duty_cycle = int(pin_config["default"] + pwm * diff)

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
