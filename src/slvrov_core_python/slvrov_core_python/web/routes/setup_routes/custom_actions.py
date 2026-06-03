import json
from flask import Blueprint, request, jsonify  # type: ignore


custom_actions_bp = Blueprint('custom_actions', __name__, url_prefix='/custom-actions')


@custom_actions_bp.route("/create_actions", methods=["POST"])
def create_actions():
    """
    * Accept req with action names and types and custom action file (add to existing file or create new one)
    * Write to custom action file in rov_config/actions
    * Return success or error response
    """
    data = request.get_json()

    actions = data.get("actions")
    action_file = data.get("action_file")

    with open(f"rov_config/actions/{action_file}", "w") as file:
        json.dump(actions, file)
    
    # if any error occurs during this process, return error response
    return jsonify({"status": "success"}), 200


@custom_actions_bp.route("/read_actions")
def read_actions():
    """
    * Accept req with custom action file (existing file, browser should show list of existing files to choose from in dropdown)
    * Return success or error response, and if success, return list of custom actions
    """
    data = request.get_json()
    action_file = data.get("action_file")

    try:
        with open(f"rov_config/actions/{action_file}", "r") as file:
            actions = json.load(file)

        return jsonify({"status": "success", "actions": actions}), 200
    
    except Exception as exception:
        return jsonify({"status": "error", "message": str(exception)}), 500


@custom_actions_bp.route("/update_action", methods=["POST"])
def update_action():
    """
    * Accept req with custom action to update, along with new name and type; edits from existing custom action file in rov_config/actions
    * delete old action and add new action to custom action file in rov_config/actions
    * Return success or error response
    """
    return ...


@custom_actions_bp.route("/delete_action", methods=["POST"])
def delete_action():
    """
    * Accept req with custom action to delete; edits from existing custom action file in rov_config/actions
    * delete action from custom action file in rov_config/actions
    * Return success or error response
    """
    return ...


@custom_actions_bp.route("/delete_action_file", methods=["POST"])
def delete_action_file():
    """
    * Accept req with custom action file to delete; edits from existing custom action file in rov_config/actions
    * delete action file from rov_config/actions
    * Return success or error response
    """
    return ...