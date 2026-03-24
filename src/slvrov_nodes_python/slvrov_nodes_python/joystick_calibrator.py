#!/usr/bin/env python3
from __future__ import annotations

from dataclasses import asdict, dataclass
from enum import Enum
from pathlib import Path
from queue import Empty, SimpleQueue
from typing import Dict, List, Optional, Tuple
import json
import sys
import threading
import yaml

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Joy

from .unimplemented_features import DEFERRED_ACTIONS


JOY_TOPIC_TYPE = "sensor_msgs/msg/Joy"

# TODO: Add comments explaining each of the default constant for each parameter.
JOYSTICK_CONFIGS_PATH = "joystick_configs.yaml"
AXIS_THRESHOLD = 0.6
BUTTON_SCORE = 1.25
QUIET_SECONDS = 0.75
SETTLE_SECONDS = 1.0
DISCOVERY_PERIOD_SEC = 1.0
BIND_ORDER = [
    "forward",
    "strafe",
    "yaw",
    "heave",
    "roll",
]
SKIP_ACTIONS = []
DEFAULT_DEADZONE = 0.10
DEFAULT_SCALE = 1.0
ALLOW_REUSE_CONTROLS = False


class Action(str, Enum):
    """Enumerate the logical actions that can be calibrated."""

    FORWARD = "forward"
    STRAFE = "strafe"
    YAW = "yaw"
    HEAVE = "heave"
    ROLL = "roll"

    @property
    def prompt(self) -> str:
        """Return the operator prompt for this logical action."""
        prompts = {
            Action.FORWARD:
                "Move the control you want to use for FORWARD in the "
                "forward direction.",
            Action.STRAFE:
                "Move the control you want to use for STRAFE to the right.",
            Action.YAW:
                "Move or twist the control you want to use for YAW.",
            Action.HEAVE:
                "Move the control you want to use for HEAVE upward.",
            Action.ROLL:
                "Move the control you want to use for ROLL.",
        }
        return prompts[self]


@dataclass(frozen=True)
class JoystickMapping:
    """Record the binding from one physical control to one logical action."""

    action: str
    topic: str
    source: str
    index: int
    invert: bool = False
    deadzone: float = 0.1
    scale: float = 1.0


@dataclass(frozen=True)
class JoySnapshot:
    """Capture joystick state at the start of one calibration step."""

    axes: Tuple[float, ...]
    buttons: Tuple[int, ...]

    @classmethod
    def from_msg(cls, msg: Joy) -> "JoySnapshot":
        """Build a snapshot from a ROS Joy message."""
        return cls(
            axes=tuple(float(value) for value in msg.axes),
            buttons=tuple(int(value) for value in msg.buttons),
        )


@dataclass
class Candidate:
    """Track the strongest movement seen for one physical control."""

    topic: str
    source: str
    index: int
    score: float
    value: float
    last_update_sec: float

    @property
    def control_key(self) -> Tuple[str, str, int]:
        """Return a stable identifier for this physical control."""
        return (self.topic, self.source, self.index)


class JoystickCalibrator(Node):
    """Interactively discover joystick bindings and save them to disk."""

    def __init__(self) -> None:
        """Initialize parameters, subscriptions, and calibration state."""
        super().__init__("joystick_calibrator")

        #TODO: add docstring with parameters
        self.declare_parameter("joy_topics", [])
        self.declare_parameter("joystick_configs_path", JOYSTICK_CONFIGS_PATH)
        self.declare_parameter("axis_threshold", AXIS_THRESHOLD)
        self.declare_parameter("button_score", BUTTON_SCORE)
        self.declare_parameter("quiet_seconds", QUIET_SECONDS)
        self.declare_parameter("settle_seconds", SETTLE_SECONDS)
        self.declare_parameter("discovery_period_sec", DISCOVERY_PERIOD_SEC)
        self.declare_parameter("bind_order", BIND_ORDER)
        self.declare_parameter("skip_actions", SKIP_ACTIONS)
        self.declare_parameter("default_deadzone", DEFAULT_DEADZONE)
        self.declare_parameter("default_scale", DEFAULT_SCALE)
        self.declare_parameter("allow_reuse_controls", ALLOW_REUSE_CONTROLS)

        # Read parameters and initialize runtime state.
        self.configured_joy_topics = [str(topic) for topic in self.get_parameter("joy_topics").value]
        self.output_path = str(self.get_parameter("joystick_configs_path").value)
        self.axis_threshold = float(self.get_parameter("axis_threshold").value)
        self.button_score = float(self.get_parameter("button_score").value)
        self.quiet_seconds = float(self.get_parameter("quiet_seconds").value)
        self.settle_seconds = float(self.get_parameter("settle_seconds").value)
        self.discovery_period_sec = float(self.get_parameter("discovery_period_sec").value)
        self.default_deadzone = float(self.get_parameter("default_deadzone").value)
        self.default_scale = float(self.get_parameter("default_scale").value)
        self.allow_reuse_controls = bool(self.get_parameter("allow_reuse_controls").value)

        bind_order_raw = [action for action in self._normalize_configured_action_names(self.get_parameter("bind_order").value,"bind_order",)]
      
        skip_actions_raw = {Action(action_name) for action_name in self._normalize_configured_action_names(self.get_parameter("skip_actions").value,"skip_actions",)}
        
        self.actions_to_bind = [
            Action(action_name)
            for action_name in bind_order_raw
            if Action(action_name) not in skip_actions_raw
        ]
        self.skipped_actions_configured = sorted(
            action.value for action in skip_actions_raw
        )
        self.skipped_actions_runtime: List[str] = []

        self.joy_topics: List[str] = []
        self.joy_subscriptions: Dict[str, object] = {}
        self.latest: Dict[str, Optional[Joy]] = {}
        self.previous: Dict[str, Optional[Joy]] = {}
        self.mappings: List[JoystickMapping] = []
        self.bound_controls: set[Tuple[str, str, int]] = set()

        self.current_action_index = 0
        self.prompt_active = False
        self.finished = False
        self.next_prompt_not_before_sec = 0.0
        self.prompt_started_sec = 0.0
        self.last_activity_sec = 0.0
        self.baselines: Dict[str, JoySnapshot] = {}
        self.candidates: Dict[Tuple[str, str, int], Candidate] = {}
        self.waiting_for_topics_logged = False
        self.waiting_for_messages_logged = False

        self.command_queue: SimpleQueue[str] = SimpleQueue()
        self.shutdown_command_prompt = False
        self.spin_command_prompt_thread = threading.Thread(
            target=self.spin_command_prompt,
            daemon=True,
        )
        self.spin_command_prompt_thread.start()

        self.discovery_timer = self.create_timer(
            self.discovery_period_sec,
            self._refresh_joy_subscriptions,
        )
        self.calibration_timer = self.create_timer(0.05, self._tick)

        self.get_logger().info(
            "Joystick calibrator started. Type 'skip' to skip the current "
            "action, 'undo' to remove the last binding, or 'quit' to save "
            "progress and exit."
        )
        if self.skipped_actions_configured:
            self.get_logger().info(
                "Actions skipped by configuration: "
                f"{self.skipped_actions_configured}"
            )

        if not self.actions_to_bind:
            self.get_logger().info(
                "All actions were skipped by configuration. Saving an empty "
                "mapping file."
            )
            self._finish(save_progress=True)
            return

        self._refresh_joy_subscriptions()

    def spin_command_prompt(self) -> None:
        """Run the terminal command loop on a background thread."""
        if not sys.stdin or not sys.stdin.isatty():
            return

        while not self.shutdown_command_prompt:
            try:
                line = input()
                self.command_queue.put(line.strip().lower())

            except EOFError:
                self.get_logger().info("End of input reached. Exiting...")
                self.shutdown_command_prompt = True

            except Exception as exception:
                self.get_logger().error(
                    f"An unexpected command prompt error occurred: {exception}"
                )

    def _normalize_configured_action_names(self,raw_values: List[object],parameter_name: str,) -> List[str]:
        """Normalize configured action names and ignore deferred ones."""
        names = [str(value) for value in raw_values]
        deferred = sorted(set(names) & set(DEFERRED_ACTIONS))
        if deferred:
            self.get_logger().warning(
                f"Ignoring deferred action(s) in parameter '{parameter_name}': "
                f"{deferred}"
            )

        active_action_names = {action.value for action in Action}
        normalized: List[str] = []
        for name in names:
            if name in DEFERRED_ACTIONS:
                continue
            if name not in active_action_names:
                raise ValueError(
                    f"Unsupported action '{name}' in parameter "
                    f"'{parameter_name}'"
                )
            normalized.append(name)

        return normalized

    def _refresh_joy_subscriptions(self) -> None:
        """Subscribe to the configured or discovered Joy topics."""
        if self.finished: return

        if self.configured_joy_topics:
            target_topics = list(dict.fromkeys(self.configured_joy_topics))
        else:
            # looks at available topics and add if are joystick
            target_topics = sorted(
                topic_name
                for topic_name, topic_types in self.get_topic_names_and_types()
                if JOY_TOPIC_TYPE in topic_types
            )

        if not target_topics:
            if not self.waiting_for_topics_logged:
                self.get_logger().info(
                    "Waiting for one or more Joy topics to appear..."
                )
                self.waiting_for_topics_logged = True
            return

        self.waiting_for_topics_logged = False

        for topic in target_topics:
            if topic in self.joy_subscriptions:
                continue

            self.latest[topic] = None
            self.previous[topic] = None
            self.joy_subscriptions[topic] = self.create_subscription(
                Joy,
                topic,
                lambda msg, bound_topic=topic: self._joy_callback(
                    bound_topic, msg
                ),
                10,
            )
            self.joy_topics.append(topic)
            self.get_logger().info(f"Subscribed to Joy topic: {topic}")

    def _joy_callback(self, topic: str, msg: Joy) -> None:
        """Track incoming joystick messages and update the active prompt."""
        previous_msg = self.latest[topic]
        self.previous[topic] = previous_msg
        self.latest[topic] = msg

        if self.prompt_active:
            self._update_candidates(topic, previous_msg, msg)

    def _tick(self) -> None:
        """Drive the calibration state machine."""
        if self.finished: return

        self._handle_user_commands()
        if self.finished: return

        if not self.joy_topics: return

        if not self._has_any_messages():
            if not self.waiting_for_messages_logged:
                self.get_logger().info(
                    "Waiting for Joy messages. Make sure joy_node is running "
                    "and your controller is publishing."
                )
                self.waiting_for_messages_logged = True
            return

        self.waiting_for_messages_logged = False

        if self.current_action_index >= len(self.actions_to_bind):
            self._finish(save_progress=True)
            return

        now_sec = self._now_sec()
        if not self.prompt_active:
            if now_sec < self.next_prompt_not_before_sec: return

            self._start_current_prompt()
            return

        best = self._best_candidate()
        if best is None:
            return

        if not self._candidate_is_ready(best, now_sec):
            return

        self._bind_candidate(best)

    def _handle_user_commands(self) -> None:
        """Apply terminal commands entered while the node is running."""
        while True:
            try:
                command = self.command_queue.get_nowait()

            except Empty: return

            if command in {"", "help", "h", "?"}:
                self.get_logger().info(
                    "Commands: 'skip', 'undo', 'quit'."
                )
                continue

            if command in {"skip", "s"}:
                if self.current_action_index >= len(self.actions_to_bind): continue

                action = self.actions_to_bind[self.current_action_index]
                self.skipped_actions_runtime.append(action.value)
                self.get_logger().info(f"Skipped action: {action.value}")
                self._advance_to_next_prompt()
                continue

            if command in {"undo", "u"}:
                if not self.mappings:
                    self.get_logger().info("Nothing to undo yet.")
                    continue
                removed = self.mappings.pop()
                self.bound_controls.discard(
                    (removed.topic, removed.source, removed.index)
                )
                if self.current_action_index > 0:
                    self.current_action_index -= 1
                self.prompt_active = False
                self.next_prompt_not_before_sec = self._now_sec()
                self.get_logger().info(
                    "Removed last binding: "
                    f"{removed.action} from {removed.topic} "
                    f"{removed.source}[{removed.index}]"
                )
                continue

            if command in {"quit", "q", "exit"}:
                self.get_logger().info(
                    "Saving current progress and shutting down calibration."
                )
                self._finish(save_progress=True)
                return

            self.get_logger().info(
                f"Unknown command '{command}'. Commands: skip, undo, quit."
            )

    def _has_any_messages(self) -> bool:
        """Return whether at least one subscribed topic has published."""
        return any(message is not None for message in self.latest.values())

    def _start_current_prompt(self) -> None:
        """Create a fresh baseline and prompt the operator for one action."""
        self.baselines = {}
        for topic, message in self.latest.items():
            if message is None:
                continue
            self.baselines[topic] = JoySnapshot.from_msg(message)

        if not self.baselines:
            return

        self.candidates = {}
        self.prompt_active = True
        now_sec = self._now_sec()
        self.prompt_started_sec = now_sec
        self.last_activity_sec = now_sec

        action = self.actions_to_bind[self.current_action_index]
        self.get_logger().info(
            f"[{self.current_action_index + 1}/{len(self.actions_to_bind)}] "
            f"{action.prompt}"
        )

    def _update_candidates(
        self,
        topic: str,
        previous_msg: Optional[Joy],
        current_msg: Joy,
    ) -> None:
        """Update detected axis and button movement for one Joy topic."""
        baseline = self.baselines.get(topic)
        if baseline is None:
            self.baselines[topic] = JoySnapshot.from_msg(current_msg)
            return

        now_sec = self._now_sec()

        axis_count = len(current_msg.axes)
        for index in range(axis_count):
            current_value = float(current_msg.axes[index])
            baseline_value = baseline.axes[index] if index < len(baseline.axes) else 0.0

            previous_value = baseline_value
            if previous_msg is not None and index < len(previous_msg.axes):
                previous_value = float(previous_msg.axes[index])

            if abs(current_value - previous_value) > 0.05:
                self.last_activity_sec = now_sec

            signed_delta = current_value - baseline_value
            score = abs(signed_delta)
            if score <= 0.0:
                continue

            key = (topic, "axis", index)
            candidate = self.candidates.get(key)
            if candidate is None or score > candidate.score:
                self.candidates[key] = Candidate(
                    topic=topic,
                    source="axis",
                    index=index,
                    score=score,
                    value=signed_delta,
                    last_update_sec=now_sec,
                )

        button_count = len(current_msg.buttons)
        for index in range(button_count):
            baseline_value = (
                baseline.buttons[index] if index < len(baseline.buttons) else 0
            )

            previous_value = baseline_value
            if previous_msg is not None and index < len(previous_msg.buttons):
                previous_value = int(previous_msg.buttons[index])

            current_value = int(current_msg.buttons[index])
            if current_value != previous_value:
                self.last_activity_sec = now_sec

            if baseline_value == 0 and previous_value == 0 and current_value == 1:
                key = (topic, "button", index)
                self.candidates[key] = Candidate(
                    topic=topic,
                    source="button",
                    index=index,
                    score=self.button_score,
                    value=1.0,
                    last_update_sec=now_sec,
                )

    def _best_candidate(self) -> Optional[Candidate]:
        """Return the strongest valid candidate seen for this prompt."""
        best: Optional[Candidate] = None
        for candidate in self.candidates.values():
            if (
                not self.allow_reuse_controls
                and candidate.control_key in self.bound_controls
            ):
                continue
            if best is None or candidate.score > best.score:
                best = candidate
        return best

    def _candidate_is_ready(self, candidate: Candidate, now_sec: float) -> bool:
        """Return whether enough motion has happened to finalize a binding."""
        min_score = (
            self.axis_threshold
            if candidate.source == "axis"
            else self.button_score
        )
        if candidate.score < min_score:
            return False

        if (now_sec - self.prompt_started_sec) < 0.2:
            return False

        if (now_sec - self.last_activity_sec) < self.quiet_seconds:
            return False

        return True

    def _bind_candidate(self, candidate: Candidate) -> None:
        """Persist one candidate as the binding for the current action."""
        action = self.actions_to_bind[self.current_action_index]
        mapping = JoystickMapping(
            action=action.value,
            topic=candidate.topic,
            source=candidate.source,
            index=candidate.index,
            invert=(candidate.source == "axis" and candidate.value < 0.0),
            deadzone=self.default_deadzone,
            scale=self.default_scale,
        )
        self.mappings.append(mapping)
        self.bound_controls.add(candidate.control_key)

        self.get_logger().info(
            "Bound "
            f"{mapping.action} -> {mapping.topic} "
            f"{mapping.source}[{mapping.index}] "
            f"(invert={mapping.invert})"
        )

        self._advance_to_next_prompt()

    def _advance_to_next_prompt(self) -> None:
        """Clear prompt-local state and move to the next action."""
        self.current_action_index += 1
        self.prompt_active = False
        self.baselines = {}
        self.candidates = {}
        self.next_prompt_not_before_sec = self._now_sec() + self.settle_seconds

    def _save_output(self) -> None:
        """Write the calibration result to JSON or YAML."""
        path = Path(self.output_path)
        path.parent.mkdir(parents=True, exist_ok=True)

        payload = {
            "joy_topics": list(self.joy_topics),
            "mappings": [asdict(mapping) for mapping in self.mappings],
        }

        with path.open("w", encoding="utf-8") as handle:
            if path.suffix.lower() == ".json":
                json.dump(payload, handle, indent=2)
                handle.write("\n")
            else:
                yaml.safe_dump(payload, handle, sort_keys=False)

    def _finish(self, save_progress: bool) -> None:
        """Optionally save the current progress and stop the node."""
        if save_progress:
            self._save_output()
            self.get_logger().info(
                f"Saved {len(self.mappings)} mapping(s) to {self.output_path}"
            )
            if self.skipped_actions_configured:
                self.get_logger().info(
                    "Configured skipped actions were not persisted to disk yet: "
                    f"{self.skipped_actions_configured}"
                )
            if self.skipped_actions_runtime:
                self.get_logger().info(
                    "Runtime-skipped actions were not persisted to disk yet: "
                    f"{sorted(set(self.skipped_actions_runtime))}"
                )

        self.shutdown_command_prompt = True
        self.finished = True
        if rclpy.ok():
            rclpy.shutdown()

    def _now_sec(self) -> float:
        """Return the ROS clock time in seconds."""
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
