import rclpy  # type: ignore
from rclpy.executors import ExternalShutdownException  # type: ignore
from rclpy.node import Node  # type: ignore
from std_srvs.srv import Trigger  # type: ignore

import gpiozero import OutputDevice  # type: ignore


class RelayNode(Node):
    def __init__(self):
        super().__init__("relay_node")

        self.declare_parameter("gpio_number", 24)
        self.gpio_pin = int(self.get_parameter("gpio_number").value)

        self.relay = OutputDevice(self.gpio_pin, active_high=True, initial_value=False)
        self.relay_on = False

        self.toggle_relay_service = self.create_service(Trigger, "toggle_relay", self.toggle_relay_callback)

    def toggle_relay_callback(self, req, resp):
        if self.relay_on: 
            self.relay.off()
            msg = "Turned relay off."
        else: 
            self.relay.on()
            msg = "Turned relay on."

        resp.success = True
        resp.message = msg
        return resp
