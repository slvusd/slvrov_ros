import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

import threading

from slvrov_interfaces.srv import AddPCA9685PinConfigs, GetPCA9685PinConfigs

# AddPCA9685PinConfigs.srv
# string id
# uint8[] pins
# uint32 minimum
# uint32 pwm_default
# uint32 maximum
# ---
# bool success
# string msg

# GetPCA9685PinConfigs.srv
# ---
# bool success
# string configs


class PinConfigsClient(Node):
    """Provide an interactive client for PCA9685 pin-config services."""

    def __init__(self):
        """Connect to config services and start the terminal REPL thread."""
        
        super().__init__("pca9685_pin_configs_client")

        self.add_configs_service = self.create_client(AddPCA9685PinConfigs, "add_pca9685_pin_configs")
        self.get_configs_service = self.create_client(GetPCA9685PinConfigs, "get_pca9685_pin_configs")

        # Wait for both services before starting the REPL so user commands never
        # queue against a server that is still coming up.
        while not self.add_configs_service.wait_for_service(timeout_sec=3.0):
            self.get_logger().info("Waiting for add_pca9685_pin_configs service...")

        while not self.get_configs_service.wait_for_service(timeout_sec=3.0):
            self.get_logger().info("Waiting for get_pca9685_pin_configs service...")

        # input() blocks the calling thread, so the REPL runs separately while
        # the executor keeps spinning to receive service responses.
        self.shutdown_repl = False
        self.spin_repl_thread = threading.Thread(target=self.spin_repl, daemon=True)
        self.spin_repl_thread.start()

    def add_configs_request(self, id_name: str, pins: list[int], minimum: int, pwm_default: int, maximum: int):
        """Send an asynchronous request to append a new pin config.

        Args:
            id_name: Logical name for the pin configuration entry.
            pins: PCA9685 channel indices that should receive the same command.
            minimum: Duty cycle used for full reverse or minimum travel.
            pwm_default: Neutral duty cycle for the controlled device.
            maximum: Duty cycle used for full forward or maximum travel.
        """
        req = AddPCA9685PinConfigs.Request()
        req.id = id_name
        req.pins = pins
        req.minimum = minimum
        req.pwm_default = pwm_default
        req.maximum = maximum

        self.get_logger().info(f"Sending add configs request with Id: {req.id}, Pins: {list(req.pins)}, Min, Def, Max: {req.minimum, req.pwm_default, req.maximum}")

        self.future = self.add_configs_service.call_async(req)
        self.future.add_done_callback(self.add_configs_response_callback)
    
    def add_configs_response_callback(self, future):
        """Log the result of an add-config service call.

        Args:
            future: Asynchronous ROS future that resolves to the service
                response.
        """
        try:
            resp = future.result()

            if resp.success:self.get_logger().info("Successfully added pin config")
            else: self.get_logger().info(resp.msg)

        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

    def get_configs_request(self):
        """Send an asynchronous request to fetch all stored pin configs."""
        req = GetPCA9685PinConfigs.Request()
        self.get_logger().info("Sending get configs request")

        self.future = self.get_configs_service.call_async(req)
        self.future.add_done_callback(self.get_configs_response_callback)

    def get_configs_response_callback(self, future):
        """Log the result of a get-configs service call.

        Args:
            future: Asynchronous ROS future that resolves to the service
                response.
        """
        try:
            resp = future.result()

            if resp.success: self.get_logger().info(f"Successfully retrieved pin configs:\n{resp.configs}")
            else: self.get_logger().info(resp.configs)

        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

    def spin_repl(self):
        """Run the interactive command loop for config management."""
        while not self.shutdown_repl:       
            try:
                self.get_logger().info("Enter 'add' to add pin configs, 'get' to retrieve configs, or 'exit' to quit:")
                user_input = input().strip().lower()

                if user_input == 'add':
                    try:
                        self.get_logger().info("Enter id/name for the pin config:")
                        id_name = input().strip()

                        self.get_logger().info("Enter pins as comma separated values (e.g. 0,1,2):")
                        pins_input = input().strip()
                        pins = [int(pin.strip()) for pin in pins_input.split(',') if pin.strip().isdigit()]

                        if not pins:
                            self.get_logger().warning("No valid pins provided. Please enter at least one pin.")
                            continue

                        # The PCA9685 exposes 16 channels indexed 0-15.
                        for pin in pins:
                            if pin < 0 or pin > 15:
                                self.get_logger().warning(f"Invalid pin number: {pin}. Pins must be between 0 and 15.")
                                continue

                        self.get_logger().info("Enter minimum PWM value (integer):")
                        minimum = int(input().strip())

                        self.get_logger().info("Enter default/middle PWM value (integer):")
                        pwm_default = int(input().strip())

                        self.get_logger().info("Enter maximum PWM value (integer):")
                        maximum = int(input().strip())

                        self.add_configs_request(id_name, pins, minimum, pwm_default, maximum)

                    except ValueError as e: self.get_logger().error(f"Invalid input: {e}. Please enter valid values.")
                    except Exception as e: self.get_logger().error(f"An error occurred: {e}")

                elif user_input == 'get': self.get_configs_request()
                elif user_input == 'exit':
                    self.get_logger().info("Exiting...")
                    self.shutdown_repl = True

                else: self.get_logger().warning("Invalid input. Please enter 'add', 'get', or 'exit'.")

            except EOFError:
                self.get_logger().info("End of input reached. Exiting...")
                self.shutdown_repl = True

            except Exception as e: 
                self.get_logger().error(f"An unexpected error occurred: {e}")

def main(args=None):
    # Initialize and run node
    try:
        rclpy.init()
        node = PinConfigsClient()

        # Multi-threaded spinning keeps ROS callbacks responsive while the REPL
        # thread waits on terminal input.
        executor = MultiThreadedExecutor()
        executor.add_node(node)

        executor.spin()

    except (KeyboardInterrupt, ExternalShutdownException): 
        print("Shutdown signal received, exiting...")

    # Destroy node (now) and gracefully exit
    finally:
        if node is not None: 
            node.shutdown_repl = True  # Signal the REPL thread to exit
            node.destroy_node()

        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__':
    main()
