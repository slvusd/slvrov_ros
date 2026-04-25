from dataclasses import dataclass
from enum import Enum
from pathlib import Path


class ROVActionType(Enum, str):
    """Enumerate the types of logical actions that can be calibrated."""

    JS_AXIS = "axis"
    JS_BUTTON = "button"
    KEYBOARD = "keyboard"


class ROVActionName(Enum, str):
    """Enumerate the logical actions that can be calibrated."""

    FORWARD = "forward"
    STRAFE = "strafe"
    YAW = "yaw"
    HEAVE = "heave"
    ROLL = "roll"
    PITCH = "pitch"

    CLAW_OPEN = "claw_open"
    CLAW_ROTATE = "claw_rotate"
    CLAW_TILT = "claw_tilt"
    CAMERA = "camera"


@dataclass
class ROVAction:
    action: ROVActionName
    type: ROVActionType


@dataclass(frozen=True)
class ControlMapping:
    """
    Record the binding from one physical control to one logical action.
    Will be serialized to JSON for storage on disk. In most cases, loading the 
    mappings from disk will be used just as a dict object, rather than converting
    back into ControlMapping instances. 
    This class is primarily for structuring the data while preforming Create and Update operations on the mappings.
    """

    action: ROVAction
    topic: str
    source: ROVActionType
    index: int
    invert: bool = False
    deadzone: float = 0.1
    scale: float = 1.0

    def _prep_json(self) -> dict:
        """
        Return a JSON-serializable dict representation of this mapping.
        """
        
        return self.action, {"topic": self.topic, "source": self.source, "index": self.index, "invert": self.invert, "deadzone": self.deadzone, "scale": self.scale}


from json import load, dump, JSONDecodeError


def write_control_mappings(mappings: list[ControlMapping], path: Path | str, indent: int=2) -> None:
    """
    Write the given mappings to a JSON file at the given path.
    """

    if type(path) == str: path = Path(path)
    path.touch(exist_ok=True)   

    json_dict = {}

    for mapping in mappings:
        action, mapping_dict = mapping._prep_json()
        json_dict[action] = mapping_dict

    with open(path, "w") as file:
        dump(json_dict, file, indent=indent)


def append_control_mapping(mapping: ControlMapping, path: Path | str, indent: int=2) -> None:
    """Append the given mapping to a JSON file at the given path."""

    if type(path) == str: path = Path(path)
    path.touch(exist_ok=True)

    with open(path, "r+") as file:
        try: mappings_json = load(file)
        except JSONDecodeError: mappings_json = {}

        for mapping in mappings_json.values():
            action, mapping_json = mapping._prep_json()

            if action in mappings_json: raise NameError(f"Action '{action}' already exists in the mapping file. Each action can only be mapped once.")
            mappings_json[action] = mapping_json

        file.seek(0)  # Move pointer back to the very start
        dump(mappings_json, file, indent=indent)
        file.truncate()  # Clean up any leftover old data


def edit_control_mapping(mapping: ControlMapping, path: Path | str, indent: int=2) -> None:
    """Edit the given mapping in a JSON file at the given path."""

    if type(path) == str: path = Path(path)
    path.touch(exist_ok=True)

    with open(path, "r+") as file:
        try: mappings_json = load(file)
        except JSONDecodeError: mappings_json = {}

        action, mapping_dict = mapping._prep_json()

        if action not in mappings_json: raise NameError(f"Action '{action}' does not exist in the mapping file. Only existing actions can be edited.")
        mappings_json[action] = mapping_dict

        file.seek(0)  # Move pointer back to the very start
        dump(mappings_json, file, indent=indent)
        file.truncate()  # Clean up any leftover old data


def delete_control_mapping(action: ROVAction, path: Path | str, indent: int=2) -> None:
    """Delete the given mapping from a JSON file at the given path."""

    if type(path) == str: path = Path(path)
    path.touch(exist_ok=True)

    with open(path, "r+") as file:
        try: mappings_json = load(file)
        except JSONDecodeError: mappings_json = {}

        if action not in mappings_json: raise NameError(f"Action '{action}' does not exist in the mapping file. Only existing actions can be deleted.")
        del mappings_json[action]

        file.seek(0)  # Move pointer back to the very start
        dump(mappings_json, file, indent=indent)
        file.truncate()  # Clean up any leftover old data


def get_control_mappings(path: Path | str) -> Dict[ROVAction, ControlMapping]:
    """Read the control mappings from a JSON file at the given path."""

    if type(path) == str: path = Path(path)
    path.touch(exist_ok=True)

    with open(path, "r") as file:
        try: mappings_json = load(file)
        except JSONDecodeError: mappings_json = {}

        return mappings