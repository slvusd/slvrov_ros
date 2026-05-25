"""Shared value objects for ROV joystick action mapping."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Any


class ROVActionType(Enum):
    """Defines joystick input kinds that can trigger an ROV action.

    Attributes:
        JS_AXIS (ROVActionType): Joystick axis input.
        JS_BUTTON (ROVActionType): Joystick button input.
    """

    JS_AXIS = "js_axis"
    JS_BUTTON = "js_button"

    def __str__(self) -> str:
        return self.value


@dataclass(frozen=True)
class ROVAction:
    """Represents a named ROV action and its expected joystick input kind.

    Attributes:
        name (str): Action name used by mappings and service requests.
        action_type (ROVActionType): Joystick input kind expected for the action.

    Args:
        name (str): Action name used by mappings and service requests.
        action_type (ROVActionType): Joystick input kind expected for the action.
    """

    name: str
    action_type: ROVActionType

    def __str__(self) -> str:
        return f"{self.name}/{self.action_type}"

    @classmethod
    def from_string(cls, action_str: str) -> ROVAction:
        """Creates an action from a `name/type` string.

        Args:
            action_str (str): Action string in `name/type` format.

        Returns:
            ROVAction: Parsed action object.

        Raises:
            ValueError: If the string format or action type is invalid.
        """

        try:
            name, action_type = action_str.split("/", maxsplit=1)
        except ValueError as exception:
            raise ValueError(
                f"Action string must use format 'name/type': {action_str}"
            ) from exception

        return cls(name=name, action_type=ROVActionType(action_type))


class ROVActions(Enum):
    """Defines default ROV actions available to joystick mapping.

    Attributes:
        MOVE_SERVO (ROVAction): Generic servo movement action.
        MOVE_MOTOR (ROVAction): Generic motor movement action.
        FORWARD (ROVAction): Forward/backward translation action.
        STRAFE (ROVAction): Left/right translation action.
        YAW (ROVAction): Yaw rotation action.
        HEAVE (ROVAction): Up/down translation action.
        ROLL (ROVAction): Roll rotation action.
        PITCH (ROVAction): Pitch rotation action.
        CLAW_OPEN (ROVAction): Claw open/close button action.
        CLAW_ROTATE (ROVAction): Claw rotation button action.
        CLAW_TILT (ROVAction): Claw tilt button action.
    """

    MOVE_SERVO = ROVAction("move_servo", ROVActionType.JS_AXIS)
    MOVE_MOTOR = ROVAction("move_motor", ROVActionType.JS_AXIS)

    FORWARD = ROVAction("forward", ROVActionType.JS_AXIS)
    STRAFE = ROVAction("strafe", ROVActionType.JS_AXIS)
    YAW = ROVAction("yaw", ROVActionType.JS_AXIS)
    HEAVE = ROVAction("heave", ROVActionType.JS_AXIS)
    ROLL = ROVAction("roll", ROVActionType.JS_AXIS)
    PITCH = ROVAction("pitch", ROVActionType.JS_AXIS)

    CLAW_OPEN = ROVAction("claw_open", ROVActionType.JS_BUTTON)
    CLAW_ROTATE = ROVAction("claw_rotate", ROVActionType.JS_BUTTON)
    CLAW_TILT = ROVAction("claw_tilt", ROVActionType.JS_BUTTON)

    def __str__(self) -> str:
        return str(self.value)


@dataclass(frozen=True)
class InputSource:
    """Identifies one concrete joystick input source.

    Attributes:
        topic (str): ROS topic that publishes the joystick input.
        input_type (ROVActionType): Joystick input kind at the index.
        index (int): Axis or button index on the joystick message.

    Args:
        topic (str): ROS topic that publishes the joystick input.
        input_type (ROVActionType): Joystick input kind at the index.
        index (int): Axis or button index on the joystick message.
    """

    topic: str
    input_type: ROVActionType
    index: int

    def key(self) -> str:
        return f"{self.topic}/{self.input_type}/{self.index}"

    def __str__(self) -> str:
        return self.key()


@dataclass
class JoystickInput:
    """Stores joystick calibration and scaling for an input source.

    Attributes:
        source (InputSource): Joystick source being configured.
        invert (bool): True when the input direction should be inverted.
        min_value (float): Minimum calibrated axis value.
        mid_value (float): Neutral calibrated axis value.
        max_value (float): Maximum calibrated axis value.
        scale (float): Multiplier applied to the input value.
        deadzone (float): Input deadzone around neutral.

    Args:
        source (InputSource): Joystick source being configured.
        invert (bool, optional): Whether to invert the input. Defaults to False.
        min_value (float, optional): Minimum calibrated axis value. Defaults to -1.0.
        mid_value (float, optional): Neutral calibrated axis value. Defaults to 0.0.
        max_value (float, optional): Maximum calibrated axis value. Defaults to 1.0.
        scale (float, optional): Multiplier applied to the input. Defaults to 1.0.
        deadzone (float, optional): Input deadzone around neutral. Defaults to 0.05.
    """

    source: InputSource
    invert: bool = False
    min_value: float = -1.0
    mid_value: float = 0.0
    max_value: float = 1.0
    scale: float = 1.0
    deadzone: float = 0.05


@dataclass
class ROVActionMapping:
    """Maps an ROV action to a concrete joystick input.

    Attributes:
        action_name (str): Name of the ROV action being mapped.
        topic (str | None): Joystick topic selected for the action.
        action_type (ROVActionType): Joystick input kind selected for the action.
        index (int | None): Axis or button index selected for the action.

    Args:
        action_name (str): Name of the ROV action being mapped.
        topic (str | None): Joystick topic selected for the action.
        action_type (ROVActionType): Joystick input kind selected for the action.
        index (int | None): Axis or button index selected for the action.
    """

    action_name: str
    topic: str | None
    action_type: ROVActionType
    index: int | None

    def key(self) -> str:
        return f"{self.topic}/{self.action_type}/{self.index}"

    def to_json(self) -> dict[str, dict[str, Any]]:
        return {
            self.key(): {
                "action": self.action_name,
                "topic": self.topic,
                "type": str(self.action_type),
                "index": self.index,
            }
        }

    @classmethod
    def from_json(cls, json_dict: dict[str, Any]) -> ROVActionMapping:
        """Creates a mapping from one saved mapping JSON entry.

        Args:
            json_dict (dict[str, Any]): Mapping data or one-key saved mapping.

        Returns:
            ROVActionMapping: Parsed action mapping.

        Raises:
            KeyError: If a required mapping field is missing.
            ValueError: If the action type is invalid.
        """

        if len(json_dict) == 1:
            mapping_data = next(iter(json_dict.values()))
        else:
            mapping_data = json_dict

        return cls(
            action_name=str(mapping_data["action"]),
            topic=mapping_data.get("topic"),
            action_type=ROVActionType(str(mapping_data["type"])),
            index=mapping_data.get("index"),
        )

    def __str__(self) -> str:
        return (
            f"{self.action_name}/{self.topic}/"
            f"{self.action_type}/{self.index}"
        )


@dataclass
class MappingCandidate:
    """Tracks a potential joystick input match during a mapping run.

    Attributes:
        topic (str): ROS topic that produced the candidate input.
        action_type (ROVActionType): Joystick input kind for the candidate.
        index (int): Axis or button index for the candidate.
        initial_score (float): Initial joystick value before mapping movement.
        score_delta (float | None): Largest observed change from the initial score.

    Args:
        topic (str): ROS topic that produced the candidate input.
        action_type (ROVActionType): Joystick input kind for the candidate.
        index (int): Axis or button index for the candidate.
        initial_score (float, optional): Initial joystick value. Defaults to 0.0.
        score_delta (float | None, optional): Largest observed value change.
            Defaults to None.
    """

    topic: str
    action_type: ROVActionType
    index: int
    initial_score: float = 0.0
    score_delta: float | None = None

    def key(self) -> str:
        return f"{self.topic}/{self.action_type}/{self.index}"

    def update_score(self, new_score: float) -> None:
        """Updates the largest observed score delta.

        Args:
            new_score (float): Latest joystick value for this candidate.
        """

        score_delta = abs(new_score - self.initial_score)
        if self.score_delta is None or score_delta > self.score_delta:
            self.score_delta = score_delta

    def __str__(self) -> str:
        return self.key()
