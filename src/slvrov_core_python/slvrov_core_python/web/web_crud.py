"""Shared JSON and file CRUD helpers for Flask setup routes."""

import json
from pathlib import Path
from typing import Any, Dict, List, Tuple, Union

from flask import Response, jsonify

from ..log_messages import (
    BaseLogMessages,
    LogMessage,
    WebCrudLogMessages,
    build_error_message,
)


CONFIG_DIR_NAME = "rov_config"
CONFIG_DIR_ALIASES = {CONFIG_DIR_NAME, "rov_configs"}

FilePath = Union[str, Path]
JsonResponse = Tuple[Response, int]


def make_error_response(message: str, status_code: int = 500) -> JsonResponse:
    """Create a standard JSON failure response.

    Args:
        message (str): Human-readable error message to send to the client.
        status_code (int, optional): HTTP status code for the response.
            Defaults to 500.

    Returns:
        tuple: Flask JSON response and HTTP status code.
    """

    return jsonify({"success": False, "message": message}), status_code


def make_crud_error_response(
    operation: LogMessage,
    reason: LogMessage,
    status_code: int = 500,
) -> JsonResponse:
    """Create a JSON failure response with a formatted CRUD log message.

    Args:
        operation (BaseLogMessagesFunctions | str): Operation that failed.
        reason (BaseLogMessagesFunctions | str): Failure reason.
        status_code (int, optional): HTTP status code for the response.
            Defaults to 500.

    Returns:
        tuple: Flask JSON response and HTTP status code.
    """

    return make_error_response(
        build_error_message(operation, reason),
        status_code,
    )


def get_config_root() -> Path:
    """Find the repository-level ROV configuration directory.

    The helper walks upward from this module and returns the first
    `rov_config` directory it finds. If the directory is not present, it
    returns the current working directory's expected `rov_config` path so
    callers can still create files under the normal configuration root.

    Returns:
        Path: The resolved configuration directory path, or the expected
        fallback path under the current working directory.
    """

    for parent in Path(__file__).resolve().parents:
        candidate = parent / CONFIG_DIR_NAME
        if candidate.is_dir():
            return candidate

    return Path.cwd() / CONFIG_DIR_NAME


def resolve_file_path(file_path: FilePath) -> Path:
    """Resolve a route-provided file path into a filesystem path.

    Relative paths that start with `rov_config` or `rov_configs` are
    resolved under the repository configuration directory. Other relative
    paths are resolved from the current working directory.

    Args:
        file_path (str | Path): Absolute path or relative path supplied by a
            route handler.

    Returns:
        Path: Resolved filesystem path.

    Raises:
        ValueError: If a `rov_config`-prefixed path escapes the configuration
            directory.
    """

    path = Path(file_path)
    if path.is_absolute():
        return path

    if path.parts and path.parts[0] in CONFIG_DIR_ALIASES:
        config_root = get_config_root().resolve()
        resolved_path = (config_root / Path(*path.parts[1:])).resolve()
        resolved_path.relative_to(config_root)
        return resolved_path

    return (Path.cwd() / path).resolve()


def load_json_object(object_key: str, file_path: FilePath) -> JsonResponse:
    """Load one top-level object from a JSON file.

    Args:
        object_key (str): Top-level JSON key to read.
        file_path (str | Path): Path to the JSON file.

    Returns:
        tuple: Flask JSON response containing `success` and `object`, plus
        the HTTP status code. Failures contain `success` and `message`.
    """

    try:
        resolved_path = resolve_file_path(file_path)
        with resolved_path.open("r", encoding="utf-8") as file:
            content = json.load(file)

        if not isinstance(content, dict):
            return make_crud_error_response(
                WebCrudLogMessages.LOAD_JSON,
                BaseLogMessages.JSON_ROOT_INVALID,
                400,
            )

        return jsonify(
            {"success": True, "object": content.get(object_key)}
        ), 200

    except FileNotFoundError:
        return make_crud_error_response(
            WebCrudLogMessages.LOAD_JSON,
            BaseLogMessages.PATH_NOT_FOUND,
            404,
        )

    except PermissionError:
        return make_crud_error_response(
            WebCrudLogMessages.LOAD_JSON,
            BaseLogMessages.PERMISSION_DENIED,
        )

    except json.JSONDecodeError as exception:
        return make_crud_error_response(
            WebCrudLogMessages.LOAD_JSON,
            BaseLogMessages.INVALID_JSON + str(exception),
        )

    except (OSError, ValueError) as exception:
        return make_crud_error_response(
            WebCrudLogMessages.LOAD_JSON,
            str(exception),
        )


def write_json_object(
    object_key: str,
    value: Any,
    file_path: FilePath,
) -> JsonResponse:
    """Write one top-level object to a JSON file.

    The write replaces the entire file with a JSON object containing only
    `object_key`. Callers should pass the complete value they want stored.

    Args:
        object_key (str): Top-level JSON key to write.
        value (Any): JSON-serializable value to store under `object_key`.
        file_path (str | Path): Path to the JSON file.

    Returns:
        tuple: Flask JSON response and HTTP status code.
    """

    if not object_key:
        reason = BaseLogMessages.MISSING_FIELD + "object_key"
        return make_crud_error_response(
            WebCrudLogMessages.WRITE_JSON,
            reason,
            400,
        )

    try:
        json_text = json.dumps({object_key: value}, indent=2)
        resolved_path = resolve_file_path(file_path)
        resolved_path.parent.mkdir(parents=True, exist_ok=True)

        with resolved_path.open("w", encoding="utf-8") as file:
            file.write(json_text)

        return jsonify(
            {"success": True, "message": "Object written successfully"}
        ), 200

    except PermissionError:
        return make_crud_error_response(
            WebCrudLogMessages.WRITE_JSON,
            BaseLogMessages.PERMISSION_DENIED,
        )

    except TypeError as exception:
        message = f"Value is not JSON serializable: {str(exception)}"
        return make_crud_error_response(
            WebCrudLogMessages.WRITE_JSON,
            message,
            400,
        )

    except (OSError, ValueError) as exception:
        return make_crud_error_response(
            WebCrudLogMessages.WRITE_JSON,
            str(exception),
        )


def load_json_array(
    array_key: str,
    file_path: FilePath,
) -> Tuple[Path, Dict[str, Any], List[Dict[str, Any]]]:
    """Load and validate an array of JSON objects from a file.

    Args:
        array_key (str): Top-level JSON key expected to contain a list.
        file_path (str | Path): Path to the JSON file.

    Returns:
        tuple: Resolved file path, full JSON content, and the object array.

    Raises:
        FileNotFoundError: If `file_path` does not exist.
        PermissionError: If the file cannot be read.
        json.JSONDecodeError: If the file does not contain valid JSON.
        ValueError: If the JSON root or target array has the wrong shape.
        OSError: If another filesystem error occurs.
    """

    resolved_path = resolve_file_path(file_path)
    with resolved_path.open("r", encoding="utf-8") as file:
        content = json.load(file)

    if not isinstance(content, dict):
        raise ValueError(str(BaseLogMessages.JSON_ROOT_INVALID))

    object_array = content.get(array_key)
    if not isinstance(object_array, list):
        raise ValueError(f"{array_key} must be a list")

    if not all(isinstance(item, dict) for item in object_array):
        raise ValueError(f"{array_key} must contain JSON objects")

    return resolved_path, content, object_array


def delete_object_in_json_array(
    array_key: str,
    object_field_key: str,
    object_key: str,
    file_path: FilePath,
) -> JsonResponse:
    """Delete one object from a JSON array by field value.

    Args:
        array_key (str): Top-level JSON key containing the object array.
        object_field_key (str): Field name used to identify the object.
        object_key (str): Field value of the object to delete.
        file_path (str | Path): Path to the JSON file.

    Returns:
        tuple: Flask JSON response and HTTP status code.
    """

    try:
        resolved_path, content, object_array = load_json_array(
            array_key,
            file_path,
        )

        updated_array = [
            item for item in object_array
            if item.get(object_field_key) != object_key
        ]

        if len(updated_array) == len(object_array):
            return make_crud_error_response(
                WebCrudLogMessages.DELETE_JSON_OBJECT,
                BaseLogMessages.PATH_NOT_FOUND + "object",
                404,
            )

        content[array_key] = updated_array
        json_text = json.dumps(content, indent=2)
        with resolved_path.open("w", encoding="utf-8") as file:
            file.write(json_text)

        return jsonify(
            {"success": True, "message": "Object deleted successfully"}
        ), 200

    except FileNotFoundError:
        return make_crud_error_response(
            WebCrudLogMessages.DELETE_JSON_OBJECT,
            BaseLogMessages.PATH_NOT_FOUND,
            404,
        )

    except PermissionError:
        return make_crud_error_response(
            WebCrudLogMessages.DELETE_JSON_OBJECT,
            BaseLogMessages.PERMISSION_DENIED,
        )

    except json.JSONDecodeError as exception:
        return make_crud_error_response(
            WebCrudLogMessages.DELETE_JSON_OBJECT,
            BaseLogMessages.INVALID_JSON + str(exception),
        )

    except TypeError as exception:
        message = f"Value is not JSON serializable: {str(exception)}"
        return make_crud_error_response(
            WebCrudLogMessages.DELETE_JSON_OBJECT,
            message,
            400,
        )

    except ValueError as exception:
        return make_crud_error_response(
            WebCrudLogMessages.DELETE_JSON_OBJECT,
            str(exception),
            400,
        )

    except OSError as exception:
        return make_crud_error_response(
            WebCrudLogMessages.DELETE_JSON_OBJECT,
            str(exception),
        )


def update_object_in_json_array(
    array_key: str,
    object_field_key: str,
    object_key: str,
    updated_object: Dict[str, Any],
    file_path: FilePath,
) -> JsonResponse:
    """Replace one object in a JSON array by field value.

    Args:
        array_key (str): Top-level JSON key containing the object array.
        object_field_key (str): Field name used to identify the object.
        object_key (str): Field value of the object to replace.
        updated_object (dict): Replacement object to append after removing
            the previous object.
        file_path (str | Path): Path to the JSON file.

    Returns:
        tuple: Flask JSON response and HTTP status code.
    """

    if not isinstance(updated_object, dict):
        return make_crud_error_response(
            WebCrudLogMessages.UPDATE_JSON_OBJECT,
            BaseLogMessages.JSON_ROOT_INVALID + "updated_object",
            400,
        )

    try:
        resolved_path, content, object_array = load_json_array(
            array_key,
            file_path,
        )

        updated_array = [
            item for item in object_array
            if item.get(object_field_key) != object_key
        ]

        if len(updated_array) == len(object_array):
            return make_crud_error_response(
                WebCrudLogMessages.UPDATE_JSON_OBJECT,
                BaseLogMessages.PATH_NOT_FOUND + "object",
                404,
            )

        updated_array.append(updated_object)
        content[array_key] = updated_array
        json_text = json.dumps(content, indent=2)

        with resolved_path.open("w", encoding="utf-8") as file:
            file.write(json_text)

        return jsonify(
            {"success": True, "message": "Object updated successfully"}
        ), 200

    except FileNotFoundError:
        return make_crud_error_response(
            WebCrudLogMessages.UPDATE_JSON_OBJECT,
            BaseLogMessages.PATH_NOT_FOUND,
            404,
        )

    except PermissionError:
        return make_crud_error_response(
            WebCrudLogMessages.UPDATE_JSON_OBJECT,
            BaseLogMessages.PERMISSION_DENIED,
        )

    except json.JSONDecodeError as exception:
        return make_crud_error_response(
            WebCrudLogMessages.UPDATE_JSON_OBJECT,
            BaseLogMessages.INVALID_JSON + str(exception),
        )

    except TypeError as exception:
        message = f"Value is not JSON serializable: {str(exception)}"
        return make_crud_error_response(
            WebCrudLogMessages.UPDATE_JSON_OBJECT,
            message,
            400,
        )

    except ValueError as exception:
        return make_crud_error_response(
            WebCrudLogMessages.UPDATE_JSON_OBJECT,
            str(exception),
            400,
        )

    except OSError as exception:
        return make_crud_error_response(
            WebCrudLogMessages.UPDATE_JSON_OBJECT,
            str(exception),
        )


def rename_file(old_path: FilePath, new_path: FilePath) -> JsonResponse:
    """Rename a file after resolving both route-provided paths.

    Args:
        old_path (str | Path): Existing file path.
        new_path (str | Path): New file path.

    Returns:
        tuple: Flask JSON response and HTTP status code.
    """

    try:
        resolved_old_path = resolve_file_path(old_path)
        resolved_new_path = resolve_file_path(new_path)
        resolved_new_path.parent.mkdir(parents=True, exist_ok=True)
        resolved_old_path.rename(resolved_new_path)

        return jsonify(
            {"success": True, "message": "File renamed successfully"}
        ), 200

    except FileNotFoundError:
        return make_crud_error_response(
            BaseLogMessages.RENAME_FILE,
            BaseLogMessages.PATH_NOT_FOUND,
            404,
        )

    except PermissionError:
        return make_crud_error_response(
            BaseLogMessages.RENAME_FILE,
            BaseLogMessages.PERMISSION_DENIED,
        )

    except (OSError, ValueError) as exception:
        return make_crud_error_response(
            BaseLogMessages.RENAME_FILE,
            str(exception),
        )


def delete_file(file_path: FilePath) -> JsonResponse:
    """Delete a file after resolving its route-provided path.

    Args:
        file_path (str | Path): File path to delete.

    Returns:
        tuple: Flask JSON response and HTTP status code.
    """

    try:
        resolve_file_path(file_path).unlink()

        return jsonify(
            {"success": True, "message": "File deleted successfully"}
        ), 200

    except FileNotFoundError:
        return make_crud_error_response(
            BaseLogMessages.DELETE_FILE,
            BaseLogMessages.PATH_NOT_FOUND,
            404,
        )

    except PermissionError:
        return make_crud_error_response(
            BaseLogMessages.DELETE_FILE,
            BaseLogMessages.PERMISSION_DENIED,
        )

    except (OSError, ValueError) as exception:
        return make_crud_error_response(
            BaseLogMessages.DELETE_FILE,
            str(exception),
        )


def get_directory_files(directory_path: FilePath) -> JsonResponse:
    """List regular files in a directory.

    Args:
        directory_path (str | Path): Directory path to inspect.

    Returns:
        tuple: Flask JSON response containing `success` and `files`, plus the
        HTTP status code. Failures contain `success` and `message`.
    """

    try:
        resolved_path = resolve_file_path(directory_path)
        if not resolved_path.is_dir():
            return make_crud_error_response(
                BaseLogMessages.LIST_DIRECTORY,
                BaseLogMessages.DIRECTORY_NOT_FOUND,
                404,
            )

        files = sorted(
            file.name for file in resolved_path.iterdir()
            if file.is_file()
        )
        return jsonify({"success": True, "files": files}), 200

    except PermissionError:
        return make_crud_error_response(
            BaseLogMessages.LIST_DIRECTORY,
            BaseLogMessages.PERMISSION_DENIED,
        )

    except (OSError, ValueError) as exception:
        return make_crud_error_response(
            BaseLogMessages.LIST_DIRECTORY,
            str(exception),
        )
