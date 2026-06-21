"""Flask routes for editing saved ROV action configuration files."""

import json
from typing import Any, Mapping, Optional

from flask import Blueprint, jsonify, request

from ... import web_crud
from .... import control_objects
from ....log_messages import (
    ActionCrudLogMessages,
    BaseLogMessages,
    LogMessage,
    build_error_message,
)


action_crud_bp = Blueprint("action_crud", __name__, url_prefix="/action_crud")
future_action_crud_bp = Blueprint(
    "future_action_crud",
    __name__,
    url_prefix="/future_action_crud",
)


def make_action_error_response(
    operation: ActionCrudLogMessages,
    reason: LogMessage,
    status_code: int,
) -> web_crud.JsonResponse:
    """Create an action CRUD failure response with log-message context.

    Args:
        operation (ActionCrudLogMessages): Action CRUD operation that failed.
        reason (BaseLogMessagesFunctions | str): Failure reason.
        status_code (int): HTTP status code for the response.

    Returns:
        tuple: Flask JSON response and HTTP status code.
    """

    message = build_error_message(operation, reason)
    return web_crud.make_error_response(message, status_code)


def resolve_request_data() -> Mapping[str, Any]:
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


def validate_predefined_actions(actions: list[Any]) -> Optional[str]:
    """Validate action dictionaries against the predefined action enum.

    Args:
        actions (list): Action dictionaries from the request body.

    Returns:
        str | None: Error message when an action is invalid; otherwise, None.
    """

    for action in actions:
        if not isinstance(action, dict):
            return "Invalid action"

        try:
            control_objects.DefaultROVActions(
                (
                    action.get("action_name"),
                    control_objects.ROVActionType(action.get("action_type")),
                )
            )

        except (TypeError, ValueError):
            return "Invalid action"

    return None


@action_crud_bp.route("/add_actions", methods=["PUT"])
def add_actions() -> web_crud.JsonResponse:
    """Replace the action list in a saved action configuration file.

    The request must include `actions`, a list of predefined action
    dictionaries, and `action_file`, the destination configuration filename.
    This route replaces the full `actions` array in that file.

    Returns:
        tuple: Flask JSON response and HTTP status code.
    """

    data = resolve_request_data()
    actions = data.get("actions")
    action_file = data.get("action_file")

    if not isinstance(actions, list) or not action_file:
        reason = BaseLogMessages.MISSING_FIELD + "actions and action_file"
        return make_action_error_response(
            ActionCrudLogMessages.ADD_ACTIONS,
            reason,
            400,
        )

    validation_message = validate_predefined_actions(actions)
    if validation_message:
        return make_action_error_response(
            ActionCrudLogMessages.ADD_ACTIONS,
            validation_message,
            400,
        )

    response = web_crud.write_json_object(
        object_key="actions",
        value=actions,
        file_path=f"rov_config/actions/{action_file}",
    )
    return response


@action_crud_bp.route("/load_actions", methods=["GET"])
def load_actions() -> web_crud.JsonResponse:
    """Load actions from a saved action configuration file.

    The request must include `action_file` as a query parameter or JSON field.

    Returns:
        tuple: Flask JSON response and HTTP status code.
    """

    data = resolve_request_data()
    action_file = data.get("action_file")

    if not action_file:
        reason = BaseLogMessages.MISSING_FIELD + "action_file"
        return make_action_error_response(
            ActionCrudLogMessages.LOAD_ACTIONS,
            reason,
            400,
        )

    response = web_crud.load_json_object(
        object_key="actions",
        file_path=f"rov_config/actions/{action_file}",
    )
    return response


@action_crud_bp.route("/delete_action", methods=["DELETE"])
def delete_action() -> web_crud.JsonResponse:
    """Delete one action from a saved action configuration file.

    The request must include `action_name` and `action_file`.

    Returns:
        tuple: Flask JSON response and HTTP status code.
    """

    data = resolve_request_data()
    action_name = data.get("action_name")
    action_file = data.get("action_file")

    if not action_name or not action_file:
        reason = BaseLogMessages.MISSING_FIELD + "action_name and action_file"
        return make_action_error_response(
            ActionCrudLogMessages.DELETE_ACTION,
            reason,
            400,
        )

    response = web_crud.delete_object_in_json_array(
        array_key="actions",
        object_field_key="action_name",
        object_key=str(action_name),
        file_path=f"rov_config/actions/{action_file}",
    )
    return response


@action_crud_bp.route("/delete_action_file", methods=["DELETE"])
def delete_action_file() -> web_crud.JsonResponse:
    """Delete a saved action configuration file.

    The request must include `action_file`.

    Returns:
        tuple: Flask JSON response and HTTP status code.
    """

    data = resolve_request_data()
    action_file = data.get("action_file")

    if not action_file:
        reason = BaseLogMessages.MISSING_FIELD + "action_file"
        return make_action_error_response(
            ActionCrudLogMessages.DELETE_ACTION_FILE,
            reason,
            400,
        )

    response = web_crud.delete_file(f"rov_config/actions/{action_file}")
    return response


@action_crud_bp.route("/merge_action_files", methods=["POST"])
def merge_action_files() -> web_crud.JsonResponse:
    """Create one action configuration file from multiple source files.

    The request must include `action_files`, a list of source filenames, and
    `merged_file`, the destination filename. Source action arrays are appended
    in request order. Duplicate action names fail the merge before writing.

    Returns:
        tuple: Flask JSON response and HTTP status code.
    """

    data = resolve_request_data()
    action_files = data.get("action_files")
    merged_file = data.get("merged_file")

    if not isinstance(action_files, list) or not merged_file:
        reason = BaseLogMessages.MISSING_FIELD + "action_files and merged_file"
        return make_action_error_response(
            ActionCrudLogMessages.MERGE_ACTION_FILES,
            reason,
            400,
        )

    try:
        merged_actions = []
        for action_file in action_files:
            action_path = web_crud.resolve_file_path(
                f"rov_config/actions/{action_file}"
            )
            with action_path.open("r", encoding="utf-8") as file:
                content = json.load(file)

            if not isinstance(content, dict):
                return make_action_error_response(
                    ActionCrudLogMessages.MERGE_ACTION_FILES,
                    BaseLogMessages.JSON_ROOT_INVALID,
                    400,
                )

            source_actions = content.get("actions", [])
            if not isinstance(source_actions, list):
                return make_action_error_response(
                    ActionCrudLogMessages.MERGE_ACTION_FILES,
                    "actions must be a list",
                    400,
                )

            if not all(isinstance(action, dict) for action in source_actions):
                return make_action_error_response(
                    ActionCrudLogMessages.MERGE_ACTION_FILES,
                    "actions must contain JSON objects",
                    400,
                )

            merged_actions.extend(source_actions)

        validation_message = validate_predefined_actions(merged_actions)
        if validation_message:
            return make_action_error_response(
                ActionCrudLogMessages.MERGE_ACTION_FILES,
                validation_message,
                400,
            )

        action_names = [action.get("action_name") for action in merged_actions]
        if len(action_names) != len(set(action_names)):
            return make_action_error_response(
                ActionCrudLogMessages.MERGE_ACTION_FILES,
                "Duplicate action names found",
                400,
            )

        if not merged_actions:
            return make_action_error_response(
                ActionCrudLogMessages.MERGE_ACTION_FILES,
                "No actions to merge",
                400,
            )

        merged_path = web_crud.resolve_file_path(
            f"rov_config/actions/{merged_file}"
        )
        json_text = json.dumps({"actions": merged_actions}, indent=2)
        merged_path.parent.mkdir(parents=True, exist_ok=True)
        with merged_path.open("w", encoding="utf-8") as file:
            file.write(json_text)

        return jsonify(
            {"success": True, "message": "Action files merged successfully"}
        ), 200

    except FileNotFoundError:
        return make_action_error_response(
            ActionCrudLogMessages.MERGE_ACTION_FILES,
            BaseLogMessages.PATH_NOT_FOUND + "one or more action files",
            404,
        )

    except PermissionError:
        return make_action_error_response(
            ActionCrudLogMessages.MERGE_ACTION_FILES,
            BaseLogMessages.PERMISSION_DENIED,
            500,
        )

    except json.JSONDecodeError as exception:
        return make_action_error_response(
            ActionCrudLogMessages.MERGE_ACTION_FILES,
            BaseLogMessages.INVALID_JSON + str(exception),
            500,
        )

    except ValueError as exception:
        return make_action_error_response(
            ActionCrudLogMessages.MERGE_ACTION_FILES,
            str(exception),
            400,
        )

    except OSError as exception:
        return make_action_error_response(
            ActionCrudLogMessages.MERGE_ACTION_FILES,
            str(exception),
            500,
        )


@action_crud_bp.route("/get_action_files", methods=["GET"])
def get_action_files() -> web_crud.JsonResponse:
    """List available action configuration files.

    Returns:
        tuple: Flask JSON response containing `files` and HTTP status code.
    """

    response = web_crud.get_directory_files("rov_config/actions")
    return response


@action_crud_bp.route("/rename_action_file", methods=["PATCH"])
def rename_action_file() -> web_crud.JsonResponse:
    """Rename a saved action configuration file.

    The request must include `old_file` and `new_file`.

    Returns:
        tuple: Flask JSON response and HTTP status code.
    """

    data = resolve_request_data()
    old_file = data.get("old_file")
    new_file = data.get("new_file")

    if not old_file or not new_file:
        reason = BaseLogMessages.MISSING_FIELD + "old_file and new_file"
        return make_action_error_response(
            ActionCrudLogMessages.RENAME_ACTION_FILE,
            reason,
            400,
        )

    response = web_crud.rename_file(
        f"rov_config/actions/{old_file}",
        f"rov_config/actions/{new_file}",
    )
    return response


@future_action_crud_bp.route("/update_action", methods=["PUT"])
def update_action() -> web_crud.JsonResponse:
    """Replace one action in a saved action configuration file.

    The request must include `action_name`, `updated_action`, and
    `action_file`. The replacement object is written as the full stored
    action object.

    Returns:
        tuple: Flask JSON response and HTTP status code.
    """

    data = resolve_request_data()
    action_name = data.get("action_name")
    updated_action = data.get("updated_action")
    action_file = data.get("action_file")

    if (
        not action_name
        or not isinstance(updated_action, dict)
        or not action_file
    ):
        reason = (
            BaseLogMessages.MISSING_FIELD
            + "action_name, updated_action, and action_file"
        )
        return make_action_error_response(
            ActionCrudLogMessages.UPDATE_ACTION,
            reason,
            400,
        )

    response = web_crud.update_object_in_json_array(
        array_key="actions",
        object_field_key="action_name",
        object_key=str(action_name),
        updated_object=updated_action,
        file_path=f"rov_config/actions/{action_file}",
    )
    return response


@future_action_crud_bp.route("/create_actions", methods=["POST"])
def create_actions() -> web_crud.JsonResponse:
    """Create a custom action configuration file.

    The request must include `actions`, a list of action dictionaries, and
    `action_file`, the destination configuration filename.

    Returns:
        tuple: Flask JSON response and HTTP status code.
    """

    data = resolve_request_data()
    actions = data.get("actions")
    action_file = data.get("action_file")

    if not isinstance(actions, list) or not action_file:
        reason = BaseLogMessages.MISSING_FIELD + "actions and action_file"
        return make_action_error_response(
            ActionCrudLogMessages.CREATE_ACTIONS,
            reason,
            400,
        )

    response = web_crud.write_json_object(
        object_key="actions",
        value=actions,
        file_path=f"rov_config/actions/{action_file}",
    )
    return response
