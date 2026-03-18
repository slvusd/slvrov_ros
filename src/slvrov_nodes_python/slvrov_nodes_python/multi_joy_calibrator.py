#!/usr/bin/env python3
from __future__ import annotations

from dataclasses import dataclass, asdict
from enum import Enum
from pathlib import Path
from typing import Dict, List, Optional, Tuple
import math
import yaml

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy


class Action(str, Enum):
    """Enumerate the logical actions that can be calibrated."""

    FORWARD = "forward"
    STRAFE = "strafe"
    YAW = "yaw"
    HEAVE = "heave"
    ROLL = "roll"
    CLAW_OPEN = "claw_open"
    CLAW_ROTATE = "claw_rotate"
    CLAW_TILT = "claw_tilt"


@dataclass(frozen=True)
class JoystickMapping:
    """Record the binding from one physical control to one logical action."""

    action: str
    topic: str
    source: str          # "axis" or "button"
    index: int
    invert: bool = False
    deadzone: float = 0.1
    scale: float = 1.0


@dataclass(frozen=True)
class Candidate:
    """Represent a candidate control movement detected during calibration."""

    topic: str
    source: str          # "axis" or "button"
    index: int
    value: float
    score: float


class MultiJoyCalibrator(Node):
    """Interactively discover joystick bindings and save them to YAML."""

    def __init__(self) -> None:
        """Initialize parameters, subscriptions, and the binding timer."""
        super().__init__("multi_joy_calibrator")

        self.declare_parameter("joy_topics", ["/joy_left", "/joy_right"])
        self.declare_parameter("output_yaml", "joy_mappings.yaml")
        self.declare_parameter("axis_threshold", 0.55)
        self.declare_parameter("button_score", 1.0)
        self.declare_parameter("settle_seconds", 1.0)
        self.declare_parameter("bind_order", [
            "forward", "strafe", "yaw", "heave", "roll",
            "claw_open", "claw_rotate", "claw_tilt",
        ])
        self.declare_parameter("default_deadzone", 0.10)
        self.declare_parameter("default_scale", 1.0)

        self.joy_topics: List[str] = list(self.get_parameter("joy_topics").value)
        self.output_yaml = str(self.get_parameter("output_yaml").value)
        self.axis_threshold = float(self.get_parameter("axis_threshold").value)
        self.button_score = float(self.get_parameter("button_score").value)
        self.settle_seconds = float(self.get_parameter("settle_seconds").value)
        self.default_deadzone = float(self.get_parameter("default_deadzone").value)
        self.default_scale = float(self.get_parameter("default_scale").value)

        bind_order_raw = list(self.get_parameter("bind_order").value)
        self.actions_to_bind: List[Action] = [Action(x) for x in bind_order_raw]

        if not self.joy_topics:
            raise ValueError("Parameter 'joy_topics' must contain at least one topic")

        self.latest: Dict[str, Optional[Joy]] = {topic: None for topic in self.joy_topics}
        self.previous: Dict[str, Optional[Joy]] = {topic: None for topic in self.joy_topics}
        self.mappings: List[JoystickMapping] = []
        self.current_action_index = 0
        self.last_bind_time = self.get_clock().now()

        self.subscriptions = []
        for topic in self.joy_topics:
            # Capture the topic name in the lambda so each subscription updates
            # the correct previous/latest message pair.
            sub = self.create_subscription(
                Joy,
                topic,
                lambda msg, t=topic: self._joy_callback(t, msg),
                10,
            )
            self.subscriptions.append(sub)

        self.timer = self.create_timer(0.05, self._check_for_binding)

        self.get_logger().info(f"Watching joystick topics: {self.joy_topics}")
        self._print_prompt()

    def _joy_callback(self, topic: str, msg: Joy) -> None:
        """Track the previous and latest Joy message for one topic.

        Args:
            topic: Topic name that produced the message.
            msg: Incoming Joy message to cache.
        """
        self.previous[topic] = self.latest[topic]
        self.latest[topic] = msg

    def _print_prompt(self) -> None:
        """Log the instruction for the next action that should be bound."""
        if self.current_action_index >= len(self.actions_to_bind):
            self.get_logger().info("Calibration complete.")
            return

        action = self.actions_to_bind[self.current_action_index]
        self.get_logger().info(self._prompt_for_action(action))

    @staticmethod
    def _prompt_for_action(action: Action) -> str:
        """Build the operator prompt string for a logical action.

        Args:
            action: Logical action currently being calibrated.

        Returns:
            The prompt that tells the operator what to move or press.
        """
        prompts = {
            Action.FORWARD: "Bind FORWARD: move the desired control forward now.",
            Action.STRAFE: "Bind STRAFE: move the desired control right now.",
            Action.YAW: "Bind YAW: twist or move the desired yaw control now.",
            Action.HEAVE: "Bind HEAVE: move the desired control upward now.",
            Action.ROLL: "Bind ROLL: move the desired roll control now.",
            Action.CLAW_OPEN: "Bind CLAW_OPEN: press the desired button/control now.",
            Action.CLAW_ROTATE: "Bind CLAW_ROTATE: move the desired rotate control now.",
            Action.CLAW_TILT: "Bind CLAW_TILT: move the desired tilt control now.",
        }
        return prompts[action]

    def _check_for_binding(self) -> None:
        """Detect the strongest recent control change and bind it."""
        if self.current_action_index >= len(self.actions_to_bind):
            return

        elapsed = (self.get_clock().now() - self.last_bind_time).nanoseconds / 1e9
        if elapsed < self.settle_seconds:
            # Give the operator time to release the previous control before
            # listening for the next intentional movement.
            return

        best: Optional[Candidate] = None
        for topic in self.joy_topics:
            candidate = self._detect_changed_control(topic)
            if candidate is None:
                continue
            if best is None or candidate.score > best.score:
                best = candidate

        if best is None:
            return

        action = self.actions_to_bind[self.current_action_index]

        invert = False
        if best.source == "axis":
            # Negative motion during binding means we should flip that axis later.
            invert = best.value < 0.0

        mapping = JoystickMapping(
            action=action.value,
            topic=best.topic,
            source=best.source,
            index=best.index,
            invert=invert,
            deadzone=self.default_deadzone,
            scale=self.default_scale,
        )

        self.mappings.append(mapping)
        self.get_logger().info(
            f"Bound {mapping.action} -> topic={mapping.topic}, "
            f"source={mapping.source}, index={mapping.index}, invert={mapping.invert}"
        )

        self.current_action_index += 1
        self.last_bind_time = self.get_clock().now()

        if self.current_action_index >= len(self.actions_to_bind):
            self._save_yaml(self.output_yaml)
            self.get_logger().info(f"Saved mappings to: {self.output_yaml}")
        else:
            self._print_prompt()

    def _detect_changed_control(self, topic: str) -> Optional[Candidate]:
        """Find the strongest axis or button change for one topic.

        Args:
            topic: Topic name whose previous/latest messages should be compared.

        Returns:
            The best candidate control change, or `None` if nothing crossed the
            detection thresholds.
        """
        prev_msg = self.previous[topic]
        new_msg = self.latest[topic]
        if prev_msg is None or new_msg is None:
            return None

        best: Optional[Candidate] = None

        axis_count = min(len(prev_msg.axes), len(new_msg.axes))
        for i in range(axis_count):
            old = float(prev_msg.axes[i])
            new = float(new_msg.axes[i])
            delta = abs(new - old)

            # Prefer the largest absolute movement so noisy axes lose to the
            # control the operator intentionally moved.
            if delta > self.axis_threshold:
                cand = Candidate(topic=topic, source="axis", index=i, value=new, score=delta)
                if best is None or cand.score > best.score:
                    best = cand

        button_count = min(len(prev_msg.buttons), len(new_msg.buttons))
        for i in range(button_count):
            old = int(prev_msg.buttons[i])
            new = int(new_msg.buttons[i])
            if old == 0 and new == 1:
                # Buttons are scored separately because they have no analog
                # delta; rising edges represent an intentional press.
                cand = Candidate(
                    topic=topic,
                    source="button",
                    index=i,
                    value=1.0,
                    score=self.button_score,
                )
                if best is None or cand.score > best.score:
                    best = cand

        return best

    def _save_yaml(self, path_str: str) -> None:
        """Write the discovered mappings to a ROS parameter YAML file.

        Args:
            path_str: Destination path for the YAML parameter file.
        """
        path = Path(path_str)
        path.parent.mkdir(parents=True, exist_ok=True)

        data = {
            "joystick_logic_node": {
                "ros__parameters": {
                    "mappings": [asdict(m) for m in self.mappings]
                }
            }
        }

        with path.open("w", encoding="utf-8") as f:
            yaml.safe_dump(data, f, sort_keys=False)


def main() -> None:
    rclpy.init()
    node = MultiJoyCalibrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
