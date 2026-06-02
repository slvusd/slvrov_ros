"""Data objects for ROV configuration JSON."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Any

from .control_objects import ROVActionMapping


class MotorDirection(Enum):
    """Defines motor rotation directions saved in ROV JSON."""

    CLOCKWISE = "clockwise"
    CW = "clockwise"
    COUNTER_CLOCKWISE = "counter_clockwise"
    CCW = "counter_clockwise"

    def __str__(self) -> str:
        return self.value

    @classmethod
    def _missing_(cls, value: object) -> MotorDirection | None:
        if not isinstance(value, str):
            return None

        aliases = {
            "clockwise": cls.CLOCKWISE,
            "cw": cls.CLOCKWISE,
            "counter_clockwise": cls.COUNTER_CLOCKWISE,
            "ccw": cls.COUNTER_CLOCKWISE,
        }
        return aliases.get(value.lower())


@dataclass
class Motor:
    """Represents one motor entry in the ROV configuration."""

    motor_name: str
    pin: int

    default_pwm: int = 1500
    min_pwm: int = 1000
    max_pwm: int = 2000
    clamp_delta_pwm: int = 100

    direction: MotorDirection | None = None
    position: list[float] | None = None
    angle: float | None = None
    action_contributions: dict[str, float] = field(default_factory=dict)

    def to_json(self) -> dict[str, object]:
        """Converts the motor to the saved JSON shape."""

        json_dict: dict[str, object] = {
            "motor_name": self.motor_name,
            "pin": self.pin,
            "default_pwm": self.default_pwm,
            "min_pwm": self.min_pwm,
            "max_pwm": self.max_pwm,
            "clamp_delta_pwm": self.clamp_delta_pwm,
            "action_contributions": self.action_contributions,
        }

        if self.direction is not None:
            json_dict["direction"] = str(self.direction)
        if self.position is not None:
            json_dict["position"] = self.position
        if self.angle is not None:
            json_dict["angle"] = self.angle

        return json_dict

    @classmethod
    def from_json(cls, json_dict: dict[str, Any]) -> Motor:
        """Creates a motor from one saved motor JSON object."""

        direction = json_dict.get("direction")

        return cls(
            motor_name=str(json_dict["motor_name"]),
            pin=int(json_dict["pin"]),
            default_pwm=int(json_dict.get("default_pwm", 1500)),
            min_pwm=int(json_dict.get("min_pwm", 1000)),
            max_pwm=int(json_dict.get("max_pwm", 2000)),
            clamp_delta_pwm=int(json_dict.get("clamp_delta_pwm", 100)),
            direction=(
                MotorDirection(direction)
                if direction is not None
                else None
            ),
            position=(
                [float(value) for value in json_dict["position"]]
                if json_dict.get("position") is not None
                else None
            ),
            angle=(
                float(json_dict["angle"])
                if json_dict.get("angle") is not None
                else None
            ),
            action_contributions={
                str(action_name): float(contribution)
                for action_name, contribution in json_dict.get(
                    "action_contributions",
                    {},
                ).items()
            },
        )


@dataclass
class ROV:
    """Represents a complete ROV configuration."""

    actions: list[ROVActionMapping] = field(default_factory=list)
    motors: list[Motor] = field(default_factory=list)

    def to_json(self) -> dict[str, object]:
        """Converts the ROV config to the saved JSON shape."""

        return {
            "actions": [action.to_json() for action in self.actions],
            "motors": [motor.to_json() for motor in self.motors],
        }

    @classmethod
    def from_json(cls, json_dict: dict[str, Any]) -> ROV:
        """Creates an ROV config from saved JSON data."""

        return cls(
            actions=[
                ROVActionMapping.from_json(action)
                for action in json_dict.get("actions", [])
            ],
            motors=[
                Motor.from_json(motor)
                for motor in json_dict.get("motors", [])
            ],
        )
