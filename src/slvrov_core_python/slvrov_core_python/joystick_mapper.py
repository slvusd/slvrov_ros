from dataclasses import dataclass

import rclpy  # type: ignore
from rclpy.executors import ExternalShutdownException  # type: ignore
from rclpy.executors import MultiThreadedExecutor  # type: ignore
from rclpy.node import Node  # type: ignore
from sensor_msgs.msg import Joy  # type: ignore
from std_srvs.srv import Trigger  # type: ignore

from .control_objects import *


class JoystickMapper(Node):
    def __init__(self) -> None:
        super().__init__("joystick_mapper")

        self.declare_parameter("joystick_topics", [])
        self.declare_parameter("joystick_mappings_path", "")
        self.declare_parameter("update_hz", 25.0)

        self.js_topics = [str(topic) for topic in self.get_parameter("joystick_topics").value]
        self.js_mappings_path = str(self.get_parameter("joystick_mappings_path").value)
        self.update_time = 1.0 / float(self.get_parameter("update_hz").value)

        self.js_subscriptions = None
        self.js_mappings = None

        self.latest_js_states: dict[str, Joy] = dict()
        self.candidates: dict[MappingCandidate, MappingCandidate] | None = None

        self.actions: list[ROVAction] = []  # TODO
        self.current_action: ROVAction = None  # TODO
        self.action_control_service = self.create_service(..., "search_for_mapping_candidate", self.activate_callback)  #TODO

        self.active = False
        self.activate_service = self.create_service(Trigger, "activate_redundant_controls", self.activate_callback)

    def js_callback(self, topic: str, msg: Joy) -> None:
        self.latest_js_states[topic] = msg
        if self.candidates is None: self.create_candidates()

    def create_candidates(self) -> None:
        """
        When activated, Joystick Mapper will create MappingCandidates that will later be used to map actions.
        This is run once per activation.
        """
        for topic, js_state in self.latest_js_states.items():
            for index, score in enumerate(js_state.buttons):
                candidate = MappingCandidate(topic, ControlSource(ROVActionType.JS_BUTTON), index, score)
                self.candidates[candidate] = candidate

            for index, score in enumerate(js_state.axes):
                candidate = MappingCandidate(topic, ControlSource(ROVActionType.JS_AXIS), index, score)
                self.candidates[candidate] = candidate

    def clear_candidates(self) -> None:
        for candidate in self.candidates.values():
            candidate.score = None

    def update_candidates(self):
        for candidate in self.candidates.values():
            source_type = candidate.source.type
            if source_type != self.current_action.type: continue  # skip candidates that are unrelated to current action

            js_state = self.latest_js_states[candidate.topic]

            if source_type == ROVActionType.JS_AXIS: scores = js_state.axes
            elif source_type == ROVActionType.JS_BUTTON: scores = js_state.buttons
            else:  # TODO make this for when people create actions, not during mapping process
                self.get_logger().warning(f"Action type {self.current_action.type} isn't supported for mapping.")

            new_score = scores[candidate.source.index]
            if candidate.score is None: continue
            if candidate.score < new_score: candidate.score = new_score

    def get_best_candidate(self) -> MappingCandidate | None:
        best_candidate = None

        for candidate in self.candidates.values():
            if best_candidate is None: 
                best_candidate = candidate
                continue

            if best_candidate.score < candidate: ...


    def evaluate_candidates(self):
        ...

    def activate_callback(self, req, resp):
        if self.active: 
            resp = self.deactivate()
        else: 
            resp = self.activate()

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
        self.get_logger().info("Successfully subscribed to joystick topics.")

        # will put new candidates into dict and update them on timer
        self.update_timer = self.create_timer(self.update_time, self.update_candidates)

        resp.success = True
        resp.message = "Successfully subscribed to joystick topics."
        return resp
    
    def deactivate(self, resp):
        self.update_timer.cancel()
        # unsubscribe from ros2 topics
        self.candidates = None
        self.js_mappings = None

        resp.success = True
        resp.message = ""
        return resp

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
        node = JoystickMapper()

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
