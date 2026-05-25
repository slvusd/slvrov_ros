"""Small JSON file helpers used by ROS service callbacks."""

from __future__ import annotations

import json
from collections.abc import Mapping
from dataclasses import asdict, is_dataclass
from pathlib import Path
from typing import Any


def _to_json_dict(data: Mapping[str, Any] | object) -> dict[str, Any]:
    """Converts supported objects into JSON dictionaries.

    Args:
        data (Mapping[str, Any] | object): JSON-compatible mapping,
            dataclass instance, or object with `to_json()`.

    Returns:
        dict[str, Any]: Dictionary ready to write as JSON.

    Raises:
        ValueError: If the object cannot be converted to a dictionary.
    """

    if isinstance(data, Mapping):
        return dict(data)

    if hasattr(data, "to_json") and callable(data.to_json):
        json_data = data.to_json()
        if isinstance(json_data, Mapping):
            return dict(json_data)

    if is_dataclass(data) and not isinstance(data, type):
        return asdict(data)

    raise ValueError(
        f"Data of type {type(data)} is not JSON serializable. "
        "Provide a dict, dataclass instance, or object with to_json()."
    )


def load_from_json(filename: str | Path) -> dict[str, Any]:
    """Loads a JSON object from a file.

    Args:
        filename (str | Path): JSON file path.

    Returns:
        dict[str, Any]: Loaded top-level JSON object.

    Raises:
        FileNotFoundError: If the JSON file does not exist.
        ValueError: If the file is not valid JSON.
    """

    json_file = Path(filename)

    try:
        with json_file.open("r") as file:
            return json.load(file)
    except json.JSONDecodeError as exception:
        raise ValueError(
            f"JSON file {json_file} is not valid JSON."
        ) from exception


def save_to_json(data: Mapping[str, Any] | object, filename: str | Path, indent: int = 2, enforce_unique_key: bool = False, overwrite: bool = False) -> None:
    """Saves JSON data to a file.

    When `overwrite` is false, existing file data is loaded first and new data
    is merged over it. Existing keys are replaced unless `enforce_unique_key`
    is true.

    Args:
        data (Mapping[str, Any] | object): Data to save.
        filename (str | Path): JSON file path.
        indent (int, optional): JSON indentation. Defaults to 2.
        enforce_unique_key (bool, optional): Whether duplicate keys should
            raise an error. Defaults to False.
        overwrite (bool, optional): Whether to replace the whole file instead
            of merging. Defaults to False.

    Raises:
        ValueError: If data cannot be serialized or existing JSON is invalid.
        OSError: If the file cannot be written.
    """

    json_file = Path(filename)
    new_data = _to_json_dict(data)
    json_dict: dict[str, Any] = {}

    if json_file.exists() and not overwrite:
        json_dict = load_from_json(json_file)
        if not isinstance(json_dict, dict):
            raise ValueError(f"JSON file {json_file} must contain an object.")

    if enforce_unique_key:
        duplicate_keys = set(new_data).intersection(json_dict)
        if duplicate_keys:
            raise ValueError(
                f"Keys already exist in JSON file {json_file}: "
                f"{sorted(duplicate_keys)}"
            )

    json_dict.update(new_data)

    with json_file.open("w") as file:
        json.dump(json_dict, file, indent=indent)
        file.write("\n")


def update_json_key(filename: str | Path, key: str, value: Any, indent: int = 2) -> None:
    """Sets one top-level JSON key and writes the file back.

    Args:
        filename (str | Path): JSON file path.
        key (str): Top-level key to update.
        value (Any): New value for the key.
        indent (int, optional): JSON indentation. Defaults to 2.

    Raises:
        FileNotFoundError: If the JSON file does not exist.
        ValueError: If the file does not contain a JSON object.
        OSError: If the file cannot be written.
    """

    json_file = Path(filename)
    json_dict = load_from_json(json_file)
    if not isinstance(json_dict, dict):
        raise ValueError(f"JSON file {json_file} must contain an object.")

    json_dict[key] = value
    save_to_json(json_dict, json_file, indent=indent, overwrite=True)


def delete_from_json(filename: str | Path, keys: list[str], indent: int = 2) -> None:
    """Deletes top-level JSON keys and writes the file back.

    Args:
        filename (str | Path): JSON file path.
        keys (list[str]): Top-level keys to delete.
        indent (int, optional): JSON indentation. Defaults to 2.

    Raises:
        FileNotFoundError: If the JSON file does not exist.
        KeyError: If any requested key is missing.
        ValueError: If the file does not contain a JSON object.
        OSError: If the file cannot be written.
    """

    json_file = Path(filename)
    json_dict = load_from_json(json_file)
    if not isinstance(json_dict, dict):
        raise ValueError(f"JSON file {json_file} must contain an object.")

    missing_keys = [key for key in keys if key not in json_dict]
    if missing_keys:
        raise KeyError(
            f"Keys not found in JSON file {json_file}: {missing_keys}"
        )

    for key in keys:
        del json_dict[key]

    save_to_json(json_dict, json_file, indent=indent, overwrite=True)
