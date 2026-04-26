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
        return str(self)


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


@dataclass
class ROVActionMapping:
    action: ROVAction

    topic: str | None = None
    index: int | None = None

    invert: bool = False

    # Only for when action.type = ROVActionType.JS_AXIS
    min: float = -1.0
    mid: float = 0.0
    max: float = 1.0

    scale: float = 1.0
    deadzone: float = 0.0

    def __str__(self):
        return f"{self.action}/{self.topic}/{self.index} invert={self.invert} min={self.min} mid={self.mid} max={self.max} scale={self.scale} deadzone={self.deadzone}"
    
    @classmethod
    def from_string(cls, mapping_str: str) -> ROVActionMapping:
        """
        Create ROVActionMapping from string.
        Format is "name/type/topic/index invert=bool min=float mid=float max=float scale=float deadzone=float", where type is an ROVActionType Enum.
        topic and index can be "None" if not applicable (e.g. for non-Joystick actions or unmapped actions).
        invert, min, mid, max, scale, and deadzone are optional parameters that can be included in any order. 
        If not included, they will default to False for invert and -1.0, 0.0, 1.0, 1.0, and 0.0 for min, mid, max, scale, and deadzone respectively.
        """
        action_topic_index, *params_str = mapping_str.split()
        *action_str, topic_str, index_str = action_topic_index.split("/")

        action = ROVAction.from_string('/'.join(action_str))
        index = int(index_str) if index_str != "None" else None
        topic = topic_str if topic_str != "None" else None

        params = dict()
        for param in params_str:
            key, value_str = param.split("=")

            if key == "invert": value = value_str == "True"
            elif key in ["min", "mid", "max", "scale", "deadzone"]: value = float(value_str)
            else: raise ValueError(f"Invalid parameter: {key}")

            params[key] = value

        return cls(action=action, topic=topic, index=index, **params)


@dataclass
class MappingCandidate:
    topic: str
    source_type: ROVActionType
    source_index: int
    score: float | None = None

    def __str__(self):
        return f"{self.topic}/{self.source_type}/{self.source_index}/{self.score}"
    
    @classmethod
    def from_string(cls, candidate_str: str) -> 'MappingCandidate':
        """
        Create MappingCandidate from string of format "topic/type/index/score", where type is an ROVActionType Enum and score is a float or None.
        """
        topic, source_str, score_str = candidate_str.split("/")

        source_type, source_index_str = source_str.split("/")
        source_type = ROVActionType(source_type)
        source_index = int(source_index_str)

        if score_str == "None": score = None
        else: score = float(score_str)

        return cls(topic=topic, source_type=source_type, source_index=source_index, score=score)

    def __hash__(self):
        return str(self)