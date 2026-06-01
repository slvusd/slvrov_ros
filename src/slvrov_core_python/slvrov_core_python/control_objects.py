"""Shared value objects for ROV joystick action mapping."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Any


class ROVActionType(Enum):
    """Defines joystick input kinds that can trigger an ROV action."""

    AXIS = "axis"
    BUTTON = "button"

    # Backwards-compatible aliases for older service callers.
    JS_AXIS = "axis"
    JS_BUTTON = "button"

    @classmethod
    def _missing_(cls, value: object) -> ROVActionType | None:
        if not isinstance(value, str):
            return None

        aliases = {
            "js_axis": cls.AXIS,
            "axis": cls.AXIS,
            "js_button": cls.BUTTON,
            "button": cls.BUTTON,
        }
        return aliases.get(value.lower())

    def __str__(self) -> str:
        return self.value


@dataclass
class ROVActionMapping:
    """Maps one named ROV action to one joystick input."""

    action_name: str
    topic: str | None
    action_type: ROVActionType
    index: int | None
    invert: bool = False

    def key(self) -> str:
        return f"{self.topic}/{self.action_type}/{self.index}"

    def to_json(self) -> dict[str, object]:
        return {
            "action_name": self.action_name,
            "topic": self.topic,
            "action_type": str(self.action_type),
            "index": self.index,
            "invert": self.invert,
        }

    @classmethod
    def from_json(cls, json_dict: dict[str, Any]) -> ROVActionMapping:
        """Creates a mapping from one saved action JSON object.

        Args:
            json_dict (dict[str, Any]): Mapping data from the actions array.

        Returns:
            ROVActionMapping: Parsed action mapping.

        Raises:
            KeyError: If a required mapping field is missing.
            ValueError: If the action type is invalid.
        """

        return cls(
            action_name=str(json_dict["action_name"]),
            topic=json_dict.get("topic"),
            action_type=ROVActionType(str(json_dict["action_type"])),
            index=json_dict.get("index"),
            invert=bool(json_dict.get("invert", False)),
        )

    def __str__(self) -> str:
        return (
            f"{self.action_name}/{self.topic}/"
            f"{self.action_type}/{self.index}/invert={self.invert}"
        )


@dataclass
class MappingCandidate:
    """Tracks a potential joystick input match during a mapping run."""

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
