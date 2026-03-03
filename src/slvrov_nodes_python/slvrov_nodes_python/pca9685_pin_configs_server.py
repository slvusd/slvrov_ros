import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from slvrov_interfaces.srv import AddPCA9685PinConfigs, GetPCA9685PinConfigs
from slvrov_tools.pca9685 import *

from .submersed_globals import PCA9685_PIN_CONFIG_PATH


class PinConfigsServer(Node):
    def __init__(self):

        super().__init__("pca9685_pin_configs_server")

        self.configs_path = PCA9685_PIN_CONFIG_PATH
        self.add_configs_service = self.create_service(AddPCA9685PinConfigs, "add_pca9685_pin_configs", self.add_configs_callback)
        self.get_configs_service = self.create_service(GetPCA9685PinConfigs, "get_pca9685_pin_configs", self.get_configs_callback)

    def add_configs_callback(self, req, resp):
        try:
            try:
                config = PCA9685_Pin_Config(req.id, list(req.pins), req.minimum, req.pwm_default, req.maximum)
            
            # This catches NameError from the PCA9685_Pin_Config constructor. 
            # We want to avoid logging it as an unexpected server error.
            except NameError as name_error:
                resp.success = False
                resp.msg = "Id/Name already exists"

                self.get_logger().warning(f"{name_error}")
                return resp

            append_pca9685_pin_configs([config], self.configs_path)

            resp.success = True
            resp.msg = ""
            
            self.get_logger().info(f"Adding Id: {req.id}, Pins: {list(req.pins)}, Min, Def, Max: {req.minimum, req.pwm_default, req.maximum}")

        except Exception as exception:
            resp.success = False
            resp.msg = "An unknown server side error occurred."

            self.get_logger().error(f"Id: {req.id}, Pins: {list(req.pins)}, Min, Def, Max: {req.minimum, req.pwm_default, req.maximum}\nERROR: {exception}")

        return resp


    # from ChatGPT


    def get_configs_callback(self, req, resp):
        try:
            self.get_logger().info(f"Reading pin configs from: {self.configs_path}")

            configs = get_pca9685_pin_configs(self.configs_path)
            
            resp.configs = str(configs)
            resp.success = True

            self.get_logger().info("Retrieved configs")

        except FileNotFoundError as e:
            resp.success = False
            resp.configs = f"Pin configs file not found: {self.configs_path}"

            self.get_logger().warning(f"Pin configs file not found. This is a server error.")

        except Exception as e:
            resp.success = False
            resp.configs = "An unknown server side error occurred."
            
            self.get_logger().error(f"Unexpected error reading pin configs ({type(e).__name__}): {e}")

        return resp


def main(args=None):
    # Initialize and run node
    try:
        rclpy.init()
        node = PinConfigsServer()
        rclpy.spin(node)

    except (KeyboardInterrupt, ExternalShutdownException): 
        print("Shutdown signal received, exiting...")

    # Destroy node (now) and gracefully exit
    finally:
        if node is not None: node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__':
    main()