from dataclasses import dataclass
from enum import Enum


class MotorDirection(Enum):
    CLOCKWISE = 0
    CW = 0
    COUNTER_CLOCKWISE = 1
    CCW = 1


@dataclass
class Position:
    x: float | None
    y: float | None
    z: float | None


@dataclass
class Motor:
    id: str
    min: int
    default: int
    max: int
    pin: int
    clamp_delta: int = 0
    direction: MotorDirection | None = None
    position: Position | None = None


@dataclass
class Camera:
    id: str
    index: int
    width: int
    height: int
    framerate: int


@dataclass
class ROV:
    motors: list[Motor]
    cameras: list[Camera]
    