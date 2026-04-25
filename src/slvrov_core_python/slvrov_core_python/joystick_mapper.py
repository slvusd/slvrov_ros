from dataclasses import dataclass

import rclpy  # type: ignore
from rclpy.executors import ExternalShutdownException  # type: ignore
from rclpy.executors import MultiThreadedExecutor  # type: ignore
from rclpy.node import Node  # type: ignore
from sensor_msgs.msg import Joy  # type: ignore
from std_srvs.srv import Trigger  # type: ignore

from .control_objects import *


@dataclass
class MappingCandidate:
    topic: str
    source: ROVActionType
    index: int
    score: float


class JoystickMapper(Node):
    def __init__(self) -> None:
        super().__init__("joystick_mapper")

        self.declare_parameter("joystick_topics", [])
        self.declare_parameter("joystick_mappings_path", "")
        self.declare_parameter("axis_threshold", 0.6)
        self.declare_parameter("update_hz", 25.0)

        self.js_topics = [str(topic) for topic in self.get_parameter("joystick_topics").value]
        self.js_mappings_path = str(self.get_parameter("joystick_mappings_path").value)
        self.axis_threshold = float(self.get_parameter("axis_threshold").value)
        self.update_threshold = 1.0 / float(self.get_parameter("update_hz").value)

        self.js_subscriptions = None
        self.js_mappings = None

        self.last_update = None
        self.candidates = []

        self.actions: list[ROVActions] = []
        self.current_action: ROVActions = None
        self.action_control_service = self.create_service(Trigger, "search_for_mapping_candidate", self.activate_callback)

        self.active = False
        self.activate_service = self.create_service(Trigger, "activate_redundant_controls", self.activate_callback)

    def activate_callback(self, req, resp):
        if self.active:
            try:
                self.js_subscriptions = [
                self.create_subscription(
                    Joy, 
                    topic, 
                    lambda msg, bound_topic=topic: self.js_callback(bound_topic, msg), 
                    10
                    ) for topic in self.js_topics]
                
            except Exception as exception:
                self.get_logger().error(f"An unknown error occurred during subscribing to joystick topics in activation: \n{exception}")

                resp.success = False
                resp.message = "An unknown exception occurred during subscribing to joystick topics. Check logs for traceback."
                return resp

            self.active = True

            resp.success = True
            resp.message = "Successfully subscribed to joystick topics."
            return resp
        
    def activate(self, resp):
        try:
            self.js_subscriptions = [
            self.create_subscription(
                Joy, 
                topic, 
                lambda msg, bound_topic=topic: self.js_callback(bound_topic, msg), 
                10
                ) for topic in self.js_topics]
                
        except Exception as exception:
            self.get_logger().error(f"An unknown error occurred during subscribing to joystick topics in activation: \n{exception}")

            resp.success = False
            resp.message = "An unknown exception occurred during subscribing to joystick topics. Check logs for traceback."
            return resp

        self.active = True

        resp.success = True
        resp.message = "Successfully subscribed to joystick topics."
        return resp

    def js_callback(self, topic: str, msg: Joy) -> None:
        if self.now_sec() - self.last_update < self.update_threshold: return  # enforce update rate
        self.last_update = self.now_sec()

        if self.current_action is None: return  # if no action is currently being calibrated, ignore joystick messages

        if self.current_action.type == ROVActionType.AXIS:
            for idx, value in enumerate(msg.axes):
                ...

        elif self.current_action.type == ROVActionType.BUTTON:
            for idx, value in enumerate(msg.buttons):
                ...

    def get_best_candidate(self) -> MappingCandidate:
        ...

    def _candidate_is_ready(self, candidate: Candidate, now_sec: float) -> bool:
        ... # assess candidates?

    def _bind_candidate(self, candidate: Candidate) -> None:
        ...

    def _save_output(self) -> None:
        ...

    def _finish(self, save_progress: bool) -> None:
        ...

    def now_sec(self) -> float:
        """
        Return the ROS clock time in seconds.
        This is used for enforcing the update rate and timestamping candidate discoveries.
        """
        
        return self.get_clock().now().nanoseconds / 1e9


def main(args=None):
    node = None
    try:
        rclpy.init(args=args)
        node = JoystickCalibrator()

        if not node.finished and rclpy.ok():
            executor = MultiThreadedExecutor()
            executor.add_node(node)
            executor.spin()

    except (KeyboardInterrupt, ExternalShutdownException):
        print("Shutdown signal received, exiting...")
    finally:
        if node is not None:
            node.shutdown_command_prompt = True
            if not node.finished:
                node.get_logger().info(
                    "Saving current progress before exit."
                )
                node._save_output()
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
