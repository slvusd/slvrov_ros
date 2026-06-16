import json
from flask import Blueprint, request, jsonify

from .. import web_crud
from .... import control_objects


action_crud_bp = Blueprint('action_crud', __name__, url_prefix='/action_crud')

# for future actions that might be implemented later, but coded fragments now anyway
future_action_crud_bp = Blueprint('future_action_crud', __name__, url_prefix='/future_action_crud')


@action_crud_bp.route("/add_actions", methods=["POST"])
def add_actions():
    data = request.get_json()
    actions = data.get("actions")
    action_file = data.get("action_file")

    for action in actions:  # Ensure actions are part of predefined actions
        try:
            control_objects.DefaultROVActions(
                (
                    action.get("action_name"), 
                    control_objects.ROVActionType(action.get("action_type"))
                ))

        except ValueError:
            return jsonify({"success": False, "message": f"Invalid action"}), 400

    response = web_crud.write_json_object(
        object_key="actions", value=actions, file_path=f"rov_config/actions/{action_file}")
    return response
    

@action_crud_bp.route("/load_actions", methods={"POST"})
def load_actions():
    data = request.get_json()
    action_file = data.get("action_file")

    response = web_crud.load_json_object(object_key="actions", file_path=f"rov_config/actions/{action_file}")
    return response


@action_crud_bp.route("/delete_action", methods=["POST"])
def delete_action():
    data = request.get_json()
    action_name = data.get("action_name")
    action_file = data.get("action_file")

    response = web_crud.delete_object_in_json_array(
        array_key="actions", object_field_key="action_name", object_key=action_name, file_path=f"rov_config/actions/{action_file}")
    return response
    

@action_crud_bp.route("/delete_action_file", methods=["POST"])
def delete_action_file():
    data = request.get_json()
    action_file = data.get("action_file")

    response = web_crud.delete_file(f"rov_config/actions/{action_file}")
    return response
    

@action_crud_bp.route("/merge_action_files", methods=["POST"])
def merge_action_files():
    data = request.get_json()
    action_files = data.get("action_files")
    merged_file = data.get("merged_file")

    try:
        merged_actions = []
        for action_file in action_files:
            action_path = web_crud.resolve_file_path(
                f"rov_config/actions/{action_file}")
            with open(action_path, "r") as file:
                content = json.load(file)
                merged_actions.extend(content.get("actions", []))

        # throw error if duplicates are found
        action_names = [action.get("action_name") for action in merged_actions]
        if len(action_names) != len(set(action_names)):
            return jsonify({"success": False, "message": "Duplicate action names found"})
        
        if len(merged_actions) == 0:
            return jsonify({"success": False, "message": "No actions to merge"})

        merged_path = web_crud.resolve_file_path(
            f"rov_config/actions/{merged_file}")
        merged_path.parent.mkdir(parents=True, exist_ok=True)
        with open(merged_path, "w") as file:
            json.dump({"actions": merged_actions}, file, indent=2)

        return jsonify({"success": True, "message": "Action files merged successfully"}), 200

    except FileNotFoundError as exception:
        return jsonify({"success": False, "message": "One or more action files not found"}), 500

    except PermissionError as exception:
        return jsonify({"success": False, "message": "Permission denied"}), 500

    except Exception as exception:
        return jsonify({"success": False, "message": f"Unexpected error: {str(exception)}"}), 500


@action_crud_bp.route("/get_action_files")
def get_action_files():
    response = web_crud.get_directory_files("rov_config/actions")
    return response


@action_crud_bp.route("/rename_action_file", methods=["POST"])
def rename_action_file():
    data = request.get_json()
    old_file = data.get("old_file")
    new_file = data.get("new_file")

    response = web_crud.rename_file(f"rov_config/actions/{old_file}", f"rov_config/actions/{new_file}")
    return response


# allows editing of actions
# MVP won't inluce this, since user will be restricted to predefined actions
@future_action_crud_bp.route("/update_action", methods=["POST"])
def update_action():
    data = request.get_json()
    action_name = data.get("action_name")
    updated_action = data.get("updated_action")
    action_file = data.get("action_file")

    response = web_crud.update_object_in_json_array(
        array_key="actions", object_field_key="action_name", object_key=action_name, updated_object=updated_action, file_path=f"rov_config/actions/{action_file}")
    return response


# allows creation of custom actions
# MVP will be restricted to predefined actions
@future_action_crud_bp.route("/create_actions", methods=["POST"])
def create_actions():
    data = request.get_json()
    actions = data.get("actions")
    action_file = data.get("action_file")

    response = web_crud.write_json_object(
        object_key="actions", value=actions, file_path=f"rov_config/actions/{action_file}")
    return response
