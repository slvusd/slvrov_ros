#!/usr/bin/env python3
from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence
import math

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy


@dataclass
class JoystickMapping:
    """Describe how a joystick input maps to a logical control action."""

    action: str
    topic: str
    source: str          # "axis" or "button"
    index: int
    invert: bool = False
    deadzone: float = 0.1
    scale: float = 1.0


@dataclass
class ControlState:
    """Store the normalized logical control values for the vehicle and claw."""

    forward: float = 0.0
    strafe: float = 0.0
    yaw: float = 0.0
    heave: float = 0.0
    roll: float = 0.0
    claw_open: float = 0.0
    claw_rotate: float = 0.0
    claw_tilt: float = 0.0


class JoyMapper:
    """Convert raw Joy messages into a merged logical control state."""

    def __init__(self, mappings: Sequence[JoystickMapping]) -> None:
        """Initialize the mapper with the configured joystick bindings.

        Args:
            mappings: Sequence of logical action bindings to evaluate.
        """
        self._mappings = list(mappings)

    @staticmethod
    def apply_deadzone(value: float, deadzone: float) -> float:
        """Suppress small axis movement and rescale the remaining range.

        Args:
            value: Raw joystick axis value in the range [-1.0, 1.0].
            deadzone: Magnitude below which the input is treated as zero.

        Returns:
            The deadzone-adjusted axis value.
        """
        if deadzone < 0.0 or deadzone >= 1.0: raise ValueError(f"deadzone must be in [0.0, 1.0), got {deadzone}")

        if abs(value) < deadzone: return 0.0

        sign = 1.0 if value >= 0.0 else -1.0
        return sign * ((abs(value) - deadzone) / (1.0 - deadzone))

    @staticmethod
    def _get_axis(msg: Joy, index: int) -> float:
        """Read an axis value safely, defaulting to zero when out of range.

        Args:
            msg: ROS Joy message to read from.
            index: Axis index to fetch.

        Returns:
            The requested axis value or `0.0` when unavailable.
        """
        if index < 0 or index >= len(msg.axes): return 0.0
        return float(msg.axes[index])

    @staticmethod
    def _get_button(msg: Joy, index: int) -> float:
        """Read a button value safely, defaulting to zero when out of range.

        Args:
            msg: ROS Joy message to read from.
            index: Button index to fetch.

        Returns:
            The requested button value or `0.0` when unavailable.
        """
        if index < 0 or index >= len(msg.buttons):
            return 0.0
        return float(msg.buttons[index])

    def read_mapping(self, msg: Joy, mapping: JoystickMapping) -> float:
        """Extract one normalized action value from a Joy message.

        Args:
            msg: ROS Joy message containing raw axes and buttons.
            mapping: Binding description for the logical action to read.

        Returns:
            A clipped logical value in the range [-1.0, 1.0].
        """
        if mapping.source == "axis":
            value = self._get_axis(msg, mapping.index)
            value = self.apply_deadzone(value, mapping.deadzone)
        elif mapping.source == "button":
            value = self._get_button(msg, mapping.index)
        else:
            raise ValueError(f"Unsupported mapping source: {mapping.source}")

        if mapping.invert:
            value *= -1.0

        value *= mapping.scale
        return float(np.clip(value, -1.0, 1.0))

    def merge_messages(self, messages_by_topic: Dict[str, Optional[Joy]]) -> ControlState:
        """Merge the latest messages from all topics into one control state.

        Args:
            messages_by_topic: Latest Joy messages keyed by topic name. Missing
                or stale topics should map to `None`.

        Returns:
            The combined logical control state.
        """
        state = ControlState()

        # Last write wins if duplicate actions exist. The validator in the node
        # rejects duplicates, so in normal operation each action has one mapping.
        for mapping in self._mappings:
            msg = messages_by_topic.get(mapping.topic)
            if msg is None:
                continue

            value = self.read_mapping(msg, mapping)

            if mapping.action == "forward":
                state.forward = value
            elif mapping.action == "strafe":
                state.strafe = value
            elif mapping.action == "yaw":
                state.yaw = value
            elif mapping.action == "heave":
                state.heave = value
            elif mapping.action == "roll":
                state.roll = value
            elif mapping.action == "claw_open":
                state.claw_open = value
            elif mapping.action == "claw_rotate":
                state.claw_rotate = value
            elif mapping.action == "claw_tilt":
                state.claw_tilt = value
            else:
                raise ValueError(f"Unsupported action: {mapping.action}")

        return state


class ROVController:
    """Convert a logical control state into mixed thruster commands."""

    def __init__(self, mixing_matrix: np.ndarray, axis_gains: np.ndarray, thruster_inversions: np.ndarray) -> None:
        """Initialize the thruster mixer and per-axis scaling.

        Args:
            mixing_matrix: Thruster mixing matrix in control order
                `[forward, strafe, yaw, heave, roll]`.
            axis_gains: Per-axis gain multipliers applied before mixing.
            thruster_inversions: Per-thruster sign corrections.
        """
        if mixing_matrix.ndim != 2:
            raise ValueError("mixing_matrix must be 2D")
        if mixing_matrix.shape[1] != 5:
            raise ValueError(f"mixing_matrix must have 5 columns, got {mixing_matrix.shape}")

        if axis_gains.shape != (5,):
            raise ValueError(f"axis_gains must have shape (5,), got {axis_gains.shape}")

        if thruster_inversions.shape != (mixing_matrix.shape[0],):
            raise ValueError(
                "thruster_inversions length must match number of thrusters "
                f"({mixing_matrix.shape[0]}), got {thruster_inversions.shape}"
            )

        self.mixing_matrix = mixing_matrix.astype(float)
        self.axis_gains = axis_gains.astype(float)
        self.thruster_inversions = thruster_inversions.astype(float)
        self.max_thrust = 1.0

    def control_vector(self, state: ControlState) -> np.ndarray:
        """Build the controller input vector from the logical state.

        Args:
            state: Current logical vehicle control state.

        Returns:
            The gain-adjusted control vector in mixer order.
        """
        return np.array(
            [
                state.forward,
                state.strafe,
                state.yaw,
                state.heave,
                state.roll,
            ],
            dtype=float,
        ) * self.axis_gains

    def calculate_thruster_outputs(self, state: ControlState) -> np.ndarray:
        """Compute normalized thruster outputs from the current control state.

        Args:
            state: Current logical vehicle control state.

        Returns:
            Normalized thruster commands in the range [-1.0, 1.0].
        """
        controls = self.control_vector(state)
        thrusters = self.mixing_matrix @ controls
        thrusters *= self.thruster_inversions

        # Preserve the requested motion direction while scaling all thrusters
        # together if any channel would saturate.
        max_output = float(np.max(np.abs(thrusters))) if thrusters.size else 0.0
        if max_output > self.max_thrust:
            thrusters = thrusters * (self.max_thrust / max_output)

        return thrusters

    @staticmethod
    def map_to_pwm(thruster_value: float, min_pwm: int = 1100, mid_pwm: int = 1500, max_pwm: int = 1900) -> int:
        """Convert a normalized thruster command into a PWM pulse width.

        Args:
            thruster_value: Normalized thruster command in the range [-1.0, 1.0].
            min_pwm: PWM value for full reverse.
            mid_pwm: PWM value for neutral.
            max_pwm: PWM value for full forward.

        Returns:
            The PWM pulse width corresponding to the requested thrust.
        """
        thruster_value = float(np.clip(thruster_value, -1.0, 1.0))
        if thruster_value >= 0.0:
            pwm = mid_pwm + (max_pwm - mid_pwm) * thruster_value
        else:
            pwm = mid_pwm + (mid_pwm - min_pwm) * thruster_value
        return int(round(pwm))


class ClawController:
    """Translate normalized claw controls into servo PWM targets."""

    def __init__(self, neutral_pwm: int = 1500, span: int = 400) -> None:
        """Initialize the claw controller PWM center and travel span.

        Args:
            neutral_pwm: PWM value representing the neutral servo position.
            span: Maximum offset from neutral applied at full command.
        """
        self.neutral_pwm = neutral_pwm
        self.span = span

    def calculate_outputs(self, state: ControlState) -> Dict[str, int]:
        """Compute PWM outputs for the claw subsystems.

        Args:
            state: Current logical claw control state.

        Returns:
            A mapping of claw actuator names to PWM targets.
        """
        claw = self.neutral_pwm + int(round(np.clip(state.claw_open, -1.0, 1.0) * self.span))
        rotate = self.neutral_pwm + int(round(np.clip(state.claw_rotate, -1.0, 1.0) * self.span))
        tilt = self.neutral_pwm + int(round(np.clip(state.claw_tilt, -1.0, 1.0) * self.span))
        return {
            "claw": claw,
            "rotate": rotate,
            "tilt": tilt,
        }


class JoystickLogicNode(Node):
    """Merge joystick topics into vehicle and claw outputs."""

    def __init__(self) -> None:
        """Initialize parameters, subscribers, controllers, and control loop."""
        super().__init__("joystick_logic_node")

        self.declare_parameter("joy_topics", ["/joy_left", "/joy_right"])
        self.declare_parameter("loop_rate_hz", 50.0)
        self.declare_parameter("joy_timeout_sec", 0.25)
        self.declare_parameter("log_debug", True)

        self.declare_parameter("mappings", [])
        self.declare_parameter("axis_gains", [1.0, 1.0, 1.0, 1.0, 1.0])
        self.declare_parameter(
            "mixing_matrix",
            [
                [1.0,  1.0, -1.0,  0.0,  0.0],
                [1.0, -1.0,  1.0,  0.0,  0.0],
                [1.0, -1.0, -1.0,  0.0,  0.0],
                [1.0,  1.0,  1.0,  0.0,  0.0],
                [0.0,  0.0,  0.0, -1.0,  1.0],
                [0.0,  0.0,  0.0, -1.0, -1.0],
            ],
        )
        self.declare_parameter("thruster_inversions", [1.0, 1.0, 1.0, 1.0, 1.0, 1.0])

        self.joy_topics: List[str] = list(self.get_parameter("joy_topics").value)
        self.loop_rate_hz = float(self.get_parameter("loop_rate_hz").value)
        self.joy_timeout_sec = float(self.get_parameter("joy_timeout_sec").value)
        self.log_debug = bool(self.get_parameter("log_debug").value)

        mappings_raw = list(self.get_parameter("mappings").value)
        self.mappings = self._parse_mappings(mappings_raw)
        self._validate_mappings(self.mappings)

        axis_gains = np.array(self.get_parameter("axis_gains").value, dtype=float)
        mixing_matrix = np.array(self.get_parameter("mixing_matrix").value, dtype=float)
        thruster_inversions = np.array(self.get_parameter("thruster_inversions").value, dtype=float)

        self.mapper = JoyMapper(self.mappings)
        self.rov = ROVController(
            mixing_matrix=mixing_matrix,
            axis_gains=axis_gains,
            thruster_inversions=thruster_inversions,
        )
        self.claw = ClawController()

        self.latest_joy: Dict[str, Optional[Joy]] = {topic: None for topic in self.joy_topics}
        self.last_joy_time: Dict[str, Optional[float]] = {topic: None for topic in self.joy_topics}

        self.subscriptions = []
        for topic in self.joy_topics:
            # Bind the current topic into the callback closure so each
            # subscription updates the correct slot in the latest-message table.
            sub = self.create_subscription(
                Joy,
                topic,
                lambda msg, t=topic: self._joy_callback(t, msg),
                10,
            )
            self.subscriptions.append(sub)

        self.latest_control_state = ControlState()
        self.latest_thruster_outputs = np.zeros(mixing_matrix.shape[0], dtype=float)
        self.latest_thruster_pwm = [1500 for _ in range(mixing_matrix.shape[0])]
        self.latest_claw_outputs = {"claw": 1500, "rotate": 1500, "tilt": 1500}

        self.timer = self.create_timer(1.0 / self.loop_rate_hz, self._control_loop)
        self._log_counter = 0

        self.get_logger().info(f"Subscribed to topics: {self.joy_topics}")
        self.get_logger().info(f"Loaded {len(self.mappings)} mappings")

    def _joy_callback(self, topic: str, msg: Joy) -> None:
        """Store the latest Joy message and timestamp for one topic.

        Args:
            topic: Topic name that produced the message.
            msg: Incoming Joy message to cache.
        """
        self.latest_joy[topic] = msg
        self.last_joy_time[topic] = self.get_clock().now().nanoseconds / 1e9

    @staticmethod
    def _parse_mappings(raw: Sequence[dict]) -> List[JoystickMapping]:
        """Convert raw parameter dictionaries into mapping objects.

        Args:
            raw: Raw mapping parameter list from ROS.

        Returns:
            Parsed joystick mapping objects.
        """
        mappings: List[JoystickMapping] = []
        for item in raw:
            if not isinstance(item, dict):
                raise ValueError(f"Each mapping must be a dict, got: {type(item)}")

            required = {"action", "topic", "source", "index"}
            missing = required - set(item.keys())
            if missing:
                raise ValueError(f"Mapping missing keys: {sorted(missing)}")

            mappings.append(
                JoystickMapping(
                    action=str(item["action"]),
                    topic=str(item["topic"]),
                    source=str(item["source"]),
                    index=int(item["index"]),
                    invert=bool(item.get("invert", False)),
                    deadzone=float(item.get("deadzone", 0.1)),
                    scale=float(item.get("scale", 1.0)),
                )
            )
        return mappings

    def _validate_mappings(self, mappings: Sequence[JoystickMapping]) -> None:
        """Validate joystick mapping definitions before use.

        Args:
            mappings: Parsed mappings to validate.
        """
        valid_actions = {
            "forward", "strafe", "yaw", "heave", "roll",
            "claw_open", "claw_rotate", "claw_tilt",
        }
        valid_sources = {"axis", "button"}

        seen_actions = set()
        for m in mappings:
            if m.action not in valid_actions:
                raise ValueError(f"Unsupported action in mapping: {m.action}")
            if m.source not in valid_sources:
                raise ValueError(f"Unsupported source in mapping: {m.source}")
            if m.index < 0:
                raise ValueError(f"Mapping index must be >= 0, got {m.index}")
            if not (0.0 <= m.deadzone < 1.0):
                raise ValueError(f"Deadzone must be in [0.0, 1.0), got {m.deadzone}")
            if m.action in seen_actions:
                raise ValueError(f"Duplicate mapping for action: {m.action}")
            seen_actions.add(m.action)

    def _topic_is_stale(self, topic: str, now_sec: float) -> bool:
        """Check whether a topic's most recent message has timed out.

        Args:
            topic: Topic name to inspect.
            now_sec: Current clock time in seconds.

        Returns:
            `True` when the cached message is missing or older than the timeout.
        """
        t = self.last_joy_time.get(topic)
        if t is None:
            return True
        return (now_sec - t) > self.joy_timeout_sec

    def _control_loop(self) -> None:
        """Recompute logical state, thruster outputs, and claw outputs."""
        now_sec = self.get_clock().now().nanoseconds / 1e9

        filtered_msgs: Dict[str, Optional[Joy]] = {}
        any_fresh = False
        for topic in self.joy_topics:
            stale = self._topic_is_stale(topic, now_sec)
            # A stale controller should behave like a disconnected input, not
            # continue driving the vehicle with its previous command.
            filtered_msgs[topic] = None if stale else self.latest_joy[topic]
            any_fresh = any_fresh or (not stale and self.latest_joy[topic] is not None)

        if not any_fresh:
            # Fail safe to neutral when every joystick has timed out.
            state = ControlState()
        else:
            state = self.mapper.merge_messages(filtered_msgs)

        thrusters = self.rov.calculate_thruster_outputs(state)
        pwm = [self.rov.map_to_pwm(float(v)) for v in thrusters]
        claw_outputs = self.claw.calculate_outputs(state)

        self.latest_control_state = state
        self.latest_thruster_outputs = thrusters
        self.latest_thruster_pwm = pwm
        self.latest_claw_outputs = claw_outputs

        if self.log_debug:
            self._log_counter += 1
            if self._log_counter >= int(max(1.0, self.loop_rate_hz / 5.0)):
                self._log_counter = 0
                self.get_logger().info(
                    "state="
                    f"F:{state.forward:+.2f} S:{state.strafe:+.2f} Y:{state.yaw:+.2f} "
                    f"H:{state.heave:+.2f} R:{state.roll:+.2f} | "
                    f"thrusters={[round(float(x), 2) for x in thrusters]} | "
                    f"pwm={pwm} | claw={claw_outputs}"
                )


def main() -> None:
    rclpy.init()
    node = JoystickLogicNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
