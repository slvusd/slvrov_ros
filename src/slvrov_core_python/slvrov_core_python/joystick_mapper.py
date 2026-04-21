from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Joy

from .rov_action_mapping import *


@dataclass
class MappingCandidate:
    #NOTE: I think I might move this to rov_action_mappings, but really this only applies to joystick-baesd mappings
    """
    A candidate binding from one physical control to one logical action, discovered during calibration.
    """

    topic: str
    source: ROVActionType
    index: int
    score: float


class JoystickMapper(Node):
    """
    Interactively discover joystick bindings and save them to json on disk.
    """

    def __init__(self) -> None:
        """
        Initialize parameters, subscriptions, and calibration state.
        """
        super().__init__("joystick_mapper")

        self.declare_parameter("joystick_topics", [])
        self.declare_parameter("joystick_mappings_path", "joystick_mappings.json")
        self.declare_parameter("axis_threshold", 0.6)
        self.declare_parameter("update_hz", 20.0)

        self.js_topics = [str(topic) for topic in self.get_parameter("joystick_topics").value]
        self.js_mappings_path = str(self.get_parameter("joystick_mappings_path").value)
        self.axis_threshold = float(self.get_parameter("axis_threshold").value)
        self.update_seconds = 1.0 / float(self.get_parameter("update_hz").value)

        self.js_subscriptions: List[object] = []  # will be filled with subscriptions when calibration begins so node doesn't start processing joystick messages until then
        self.js_mappings: List[ControlMapping] = []

        self.last_update_sec = 0.0
        self.candidates: List[MappingCandidate] = []

        self.current_action = None

    def begin_mapping(self) -> None:
        self.get_logger().info("Beginning joystick mapping process. Move each control in turn to discover its bindings.")

        self.js_subscriptions = [
            self.create_subscription(
                Joy, 
                topic, 
                lambda msg, bound_topic=topic: self.js_callback(bound_topic, msg), 
                10
                ) for topic in self.js_topics]

        ...

    def spin_command_prompt(self) -> None:
        # TODO
        ...

    def js_callback(self, topic: str, msg: Joy) -> None:
        if self.now_sec() - self.last_update_sec < self.update_seconds: return  # enforce update rate
        self.last_update_sec = self.now_sec()

        if self.current_action is None: return  # if no action is currently being calibrated, ignore joystick messages
        elif self.current_action.source == ROVActionType.AXIS:
            for idx, value in enumerate(msg.axes):
                ...

        elif self.current_action.source == ROVActionType.BUTTON:
            for idx, value in enumerate(msg.buttons):
                ...


    def _tick(self) -> None:
        ...

    def _handle_user_commands(self) -> None:
        ...

    def _update_candidates(self,topic: str,previous_msg: Optional[Joy],current_msg: Joy,) -> None:
        ...

    def _best_candidate(self) -> Optional[Candidate]:
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
