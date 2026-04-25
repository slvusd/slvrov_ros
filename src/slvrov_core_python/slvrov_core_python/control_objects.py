from dataclasses import dataclass
from enum import Enum


class ROVActionType(Enum, str):
    JS_AXIS = "axis"
    JS_BUTTON = "button"


@dataclass
class ROVAction:
    name: str
    type: ROVActionType


class ROVActions(Enum):
    """
    Some default ROVActions created for ease of user.
    Joystick mapper will just look at ROVAction objects; it won't know about this Enum.
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


@dataclass
class ROVActionMapping:
    action: ROVActions

    topic: str
    index: int

    invert: bool = False

    # Only for when action.type = ROVActionType.JS_AXIS
    min: float = -1.0
    mid: float = 0.0
    max: float = 1.0

    scale: float = 1.0
    deadzone: float = 0.1


@dataclass
class ControlSource:
    type: ROVActionType
    index: int

    def __eq__(self, other):
        if not isinstance(other, ControlSource): raise TypeError(f"Cannot compare ControlSource with {type(other)}")
        return self.index == other.index and self.type == other.type


@dataclass
class MappingCandidate:
    topic: str
    source: ControlSource
    score: float | None = None

    def __hash__(self):
        return f"{self.topic}/{self.type}/{self.index}"