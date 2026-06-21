"""Flask routes for editing saved ROV motor configuration files."""

import json
from typing import Any, Mapping, Optional

from flask import Blueprint, jsonify, request

from ... import web_crud
from .... import rov_objects
from ....log_messages import (
    BaseLogMessages,
    LogMessage,
    MotorCrudLogMessages,
    build_error_message,
)


motor_crud_bp = Blueprint("motor_crud", __name__, url_prefix="/motor_crud")


def make_motor_error_response(
    operation: MotorCrudLogMessages,
    reason: LogMessage,
    status_code: int,
) -> web_crud.JsonResponse:
    """Create a motor CRUD failure response with log-message context.

    Args:
        operation (MotorCrudLogMessages): Motor CRUD operation that failed.
        reason (BaseLogMessagesFunctions | str): Failure reason.
        status_code (int): HTTP status code for the response.

    Returns:
        tuple: Flask JSON response and HTTP status code.
    """

    message = build_error_message(operation, reason)
    return web_crud.make_error_response(message, status_code)


def _resolve_request_data() -> Mapping[str, Any]:
    """Return request JSON fields or query parameters.

    Flask raises an unsupported-media response when `get_json()` is called on
    non-JSON requests without `silent=True`. This helper lets GET requests use
    query parameters while write requests can still send JSON bodies.

    Returns:
        Mapping: JSON body fields when the body is a JSON object; otherwise,
        query parameters for the active request.
    """

    data = request.get_json(silent=True)
    if isinstance(data, dict):
        return data

    return request.args


def _validate_motors(motors: list[Any]) -> Optional[str]:
    """Validate motor dictionaries with the Motor JSON parser.

    Args:
        motors (list): Motor dictionaries from the request body.

    Returns:
        str | None: Error message when a motor is invalid; otherwise, None.
    """

    for motor in motors:
        if not isinstance(motor, dict):
            return "Invalid motor"

        try:
            rov_objects.Motor.from_json(motor)
        except (TypeError, ValueError):
            return "Invalid motor"

    return None


@motor_crud_bp.route("/create_motors", methods=["POST"])
def add_motors() -> web_crud.JsonResponse:
    """Create a saved motor configuration file.

    The request must include `motors`, a list of motor dictionaries, and
    `motor_file`, the destination configuration filename.

    Returns:
        tuple: Flask JSON response and HTTP status code.
    """

    data = _resolve_request_data()
    motors = data.get("motors")
    motor_file = data.get("motor_file")

    if not isinstance(motors, list) or not motor_file:
        reason = BaseLogMessages.MISSING_FIELD + "motors and motor_file"
        return make_motor_error_response(
            MotorCrudLogMessages.CREATE_MOTORS,
            reason,
            400,
        )

    validation_message = _validate_motors(motors)
    if validation_message:
        return make_motor_error_response(
            MotorCrudLogMessages.CREATE_MOTORS,
            validation_message,
            400,
        )

    response = web_crud.write_json_element(
        element_key="motors",
        value=motors,
        file_path=f"rov_config/motors/{motor_file}",
    )
    return response


@motor_crud_bp.route("/load_motors", methods=["GET"])
def load_motors() -> web_crud.JsonResponse:
    """Load motors from a saved motor configuration file.

    The request must include `motor_file` as a query parameter or JSON field.

    Returns:
        tuple: Flask JSON response and HTTP status code.
    """

    data = _resolve_request_data()
    motor_file = data.get("motor_file")

    if not motor_file:
        reason = BaseLogMessages.MISSING_FIELD + "motor_file"
        return make_motor_error_response(
            MotorCrudLogMessages.LOAD_MOTORS,
            reason,
            400,
        )

    response = web_crud.load_json_element(
        element_key="motors",
        file_path=f"rov_config/motors/{motor_file}",
    )
    return response


@motor_crud_bp.route("/update_motor", methods=["PUT"])
def update_motor() -> web_crud.JsonResponse:
    """Replace one motor in a saved motor configuration file.

    The request must include `motor_name`, `updated_motor`, and `motor_file`.
    The replacement object is validated with `Motor.from_json()` before IO.

    Returns:
        tuple: Flask JSON response and HTTP status code.
    """

    data = _resolve_request_data()
    motor_name = data.get("motor_name")
    updated_motor = data.get("updated_motor")
    motor_file = data.get("motor_file")

    if not motor_name or not isinstance(updated_motor, dict) or not motor_file:
        reason = (
            BaseLogMessages.MISSING_FIELD
            + "motor_name, updated_motor, and motor_file"
        )
        return make_motor_error_response(
            MotorCrudLogMessages.UPDATE_MOTOR,
            reason,
            400,
        )

    validation_message = _validate_motors([updated_motor])
    if validation_message:
        return make_motor_error_response(
            MotorCrudLogMessages.UPDATE_MOTOR,
            validation_message,
            400,
        )

    response = web_crud.update_object_in_json_array(
        array_key="motors",
        object_field_key="motor_name",
        object_key=str(motor_name),
        updated_object=updated_motor,
        file_path=f"rov_config/motors/{motor_file}",
    )
    return response


@motor_crud_bp.route("/delete_motor", methods=["DELETE"])
def delete_motor() -> web_crud.JsonResponse:
    """Delete one motor from a saved motor configuration file.

    The request must include `motor_name` and `motor_file`.

    Returns:
        tuple: Flask JSON response and HTTP status code.
    """

    data = _resolve_request_data()
    motor_name = data.get("motor_name")
    motor_file = data.get("motor_file")

    if not motor_name or not motor_file:
        reason = BaseLogMessages.MISSING_FIELD + "motor_name and motor_file"
        return make_motor_error_response(
            MotorCrudLogMessages.DELETE_MOTOR,
            reason,
            400,
        )

    response = web_crud.delete_object_in_json_array(
        array_key="motors",
        object_field_key="motor_name",
        object_key=str(motor_name),
        file_path=f"rov_config/motors/{motor_file}",
    )
    return response


@motor_crud_bp.route("/delete_motor_file", methods=["DELETE"])
def delete_motor_file() -> web_crud.JsonResponse:
    """Delete a saved motor configuration file.

    The request must include `motor_file`.

    Returns:
        tuple: Flask JSON response and HTTP status code.
    """

    data = _resolve_request_data()
    motor_file = data.get("motor_file")

    if not motor_file:
        reason = BaseLogMessages.MISSING_FIELD + "motor_file"
        return make_motor_error_response(
            MotorCrudLogMessages.DELETE_MOTOR_FILE,
            reason,
            400,
        )

    response = web_crud.delete_file(f"rov_config/motors/{motor_file}")
    return response


@motor_crud_bp.route("/merge_motor_files", methods=["POST"])
def merge_motor_files() -> web_crud.JsonResponse:
    """Create one motor configuration file from multiple source files.

    The request must include `motor_files`, a list of source filenames, and
    `merged_file`, the destination filename. Source motor arrays are appended
    in request order. Duplicate motor names fail the merge before writing.

    Returns:
        tuple: Flask JSON response and HTTP status code.
    """

    data = _resolve_request_data()
    motor_files = data.get("motor_files")
    merged_file = data.get("merged_file")

    if not isinstance(motor_files, list) or not merged_file:
        reason = BaseLogMessages.MISSING_FIELD + "motor_files and merged_file"
        return make_motor_error_response(
            MotorCrudLogMessages.MERGE_MOTOR_FILES,
            reason,
            400,
        )

    try:
        merged_motors = []
        for motor_file in motor_files:
            motor_path = web_crud.resolve_file_path(
                f"rov_config/motors/{motor_file}"
            )
            with motor_path.open("r", encoding="utf-8") as file:
                content = json.load(file)

            if not isinstance(content, dict):
                return make_motor_error_response(
                    MotorCrudLogMessages.MERGE_MOTOR_FILES,
                    BaseLogMessages.JSON_ROOT_INVALID,
                    400,
                )

            source_motors = content.get("motors", [])
            if not isinstance(source_motors, list):
                return make_motor_error_response(
                    MotorCrudLogMessages.MERGE_MOTOR_FILES,
                    "motors must be a list",
                    400,
                )

            if not all(isinstance(motor, dict) for motor in source_motors):
                return make_motor_error_response(
                    MotorCrudLogMessages.MERGE_MOTOR_FILES,
                    "motors must contain JSON objects",
                    400,
                )

            validation_message = _validate_motors(source_motors)
            if validation_message:
                return make_motor_error_response(
                    MotorCrudLogMessages.MERGE_MOTOR_FILES,
                    validation_message,
                    400,
                )

            merged_motors.extend(source_motors)

        motor_names = [motor.get("motor_name") for motor in merged_motors]
        if len(motor_names) != len(set(motor_names)):
            return make_motor_error_response(
                MotorCrudLogMessages.MERGE_MOTOR_FILES,
                "Duplicate motor names found",
                400,
            )

        if not merged_motors:
            return make_motor_error_response(
                MotorCrudLogMessages.MERGE_MOTOR_FILES,
                "No motors to merge",
                400,
            )

        merged_path = web_crud.resolve_file_path(
            f"rov_config/motors/{merged_file}"
        )
        json_text = json.dumps({"motors": merged_motors}, indent=2)
        merged_path.parent.mkdir(parents=True, exist_ok=True)
        with merged_path.open("w", encoding="utf-8") as file:
            file.write(json_text)

        return jsonify(
            {"success": True, "message": "Motor files merged successfully"}
        ), 200

    except FileNotFoundError:
        return make_motor_error_response(
            MotorCrudLogMessages.MERGE_MOTOR_FILES,
            BaseLogMessages.PATH_NOT_FOUND + "one or more motor files",
            404,
        )

    except PermissionError:
        return make_motor_error_response(
            MotorCrudLogMessages.MERGE_MOTOR_FILES,
            BaseLogMessages.PERMISSION_DENIED,
            500,
        )

    except json.JSONDecodeError as exception:
        return make_motor_error_response(
            MotorCrudLogMessages.MERGE_MOTOR_FILES,
            BaseLogMessages.INVALID_JSON + str(exception),
            500,
        )

    except ValueError as exception:
        return make_motor_error_response(
            MotorCrudLogMessages.MERGE_MOTOR_FILES,
            str(exception),
            400,
        )

    except OSError as exception:
        return make_motor_error_response(
            MotorCrudLogMessages.MERGE_MOTOR_FILES,
            str(exception),
            500,
        )


@motor_crud_bp.route("/get_motor_files", methods=["GET"])
def get_motor_files() -> web_crud.JsonResponse:
    """List available motor configuration files.

    Returns:
        tuple: Flask JSON response containing `files` and HTTP status code.
    """

    response = web_crud.get_directory_files("rov_config/motors")
    return response


@motor_crud_bp.route("/rename_motor_file", methods=["PATCH"])
def rename_motor_file() -> web_crud.JsonResponse:
    """Rename a saved motor configuration file.

    The request must include `old_file` and `new_file`.

    Returns:
        tuple: Flask JSON response and HTTP status code.
    """

    data = _resolve_request_data()
    old_file = data.get("old_file")
    new_file = data.get("new_file")

    if not old_file or not new_file:
        reason = BaseLogMessages.MISSING_FIELD + "old_file and new_file"
        return make_motor_error_response(
            MotorCrudLogMessages.RENAME_MOTOR_FILE,
            reason,
            400,
        )

    response = web_crud.rename_file(
        f"rov_config/motors/{old_file}",
        f"rov_config/motors/{new_file}",
    )
    return response
