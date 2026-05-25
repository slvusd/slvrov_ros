from dataclasses import dataclass
from enum import Enum


class ROVActionType(Enum):
    JS_AXIS: str = "js_axis"
    JS_BUTTON: str = "js_button"

    def __str__(self):
        return self.value
    

@dataclass
class ROVAction:
    name: str
    type: ROVActionType

    def __str__(self):
        return f"{self.name}/{self.type}"
    
    @classmethod
    def from_string(cls, action_str: str) -> 'ROVAction':
        """
        Create ROVAction from string of format "name/type", where type is an ROVActionType Enum.
        """
        name, action_type = action_str.split("/")
        
        if action_type == str(ROVActionType.JS_AXIS): instance_type = ROVActionType.JS_AXIS
        elif action_type == str(ROVActionType.JS_BUTTON): instance_type = ROVActionType.JS_BUTTON
        else: raise ValueError(f"Invalid action type: {action_type}")

        return cls(name=name, type=instance_type)

    def __hash__(self):
        return hash(str(self))


class ROVActions(Enum):
    """
    Some default instances of ROVAction created for ease of user.
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

    def __str__(self):
        return str(self.value)
    

@dataclass
class InputSource:
    ...  # TODO: change these structures to use this


@dataclass
class JoystickInput:
    topic: str
    type: ROVActionType
    index: int

    invert: bool = False

    # Only for when type = ROVActionType.JS_AXIS
    min: float = -1.0
    mid: float = 0.0
    max: float = 1.0

    scale: float = 1.0
    deadzone: float = 0.05  # made this match joy_node default


@dataclass
class ROVActionMapping:
    action_name: str

    topic: str | None
    type: ROVActionType
    index: int | None

    def __str__(self):
        return f"{self.action_name}/{self.topic}/{self.type}/{self.index}"

    def __json__(self):
        return {f"{self.topic}/{self.type}/{self.index}": 
                {"action": self.action_name, 
                 "topic": self.topic,
                 "type": str(self.type), 
                 "index": self.index}}

    @classmethod
    def from_json(cls, json_dict: dict) -> 'ROVActionMapping':
        ...  # TODO: this will be used in joystick logic


@dataclass
class MappingCandidate:
    topic: str
    type: ROVActionType
    index: int
    
    initial_score: float = 0.0
    score_delta: float | None = None

    def __str__(self):
        return f"{self.topic}/{self.type}/{self.index}"

    def __hash__(self):
        return hash(str(self))