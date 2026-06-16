import json
from flask import Blueprint, request, jsonify

from .. import web_crud
from .... import rov_objects


motor_crud_bp = Blueprint('motor_crud', __name__, url_prefix='/motor_crud')


@motor_crud_bp.route("/create_motors", methods=["POST"])
def add_motors():
    data = request.get_json()
    motors = data.get("motors")
    motor_file = data.get("motor_file")

    try:
        for motor in motors:
            rov_objects.Motor.from_json(motor)
    
    except ValueError:
        return jsonify({"success": False, "message": f"Invalid motor"}), 400

    response = web_crud.write_json_element(
        element_key="motors", value=motors, file_path=f"rov_config/motors/{motor_file}")
    return response
    
    
@motor_crud_bp.route("/load_motors", methods={"POST"})
def load_motors():
    data = request.get_json()
    motor_file = data.get("motor_file")

    response = web_crud.load_json_element(element_key="motors", file_path=f"rov_config/motors/{motor_file}")
    return response
    

@motor_crud_bp.route("/update_motor", methods=["POST"])
def update_motor():
    data = request.get_json()
    motor_name = data.get("motor_name")
    updated_motor = data.get("updated_motor")
    motor_file = data.get("motor_file")

    try:
        rov_objects.Motor.from_json(updated_motor)
    
    except ValueError:
        return jsonify({"success": False, "message": f"Invalid motor"}), 400

    response = web_crud.update_object_in_json_array(
        array_key="motors", object_field_key="motor_name", object_key=motor_name, updated_object=updated_motor, file_path=f"rov_config/motors/{motor_file}")
    return response


@motor_crud_bp.route("/delete_motor", methods=["POST"])
def delete_motor():
    data = request.get_json()
    motor_name = data.get("motor_name")
    motor_file = data.get("motor_file")

    response = web_crud.delete_object_in_json_array(
        array_key="motors", object_field_key="motor_name", object_key=motor_name, file_path=f"rov_config/motors/{motor_file}")
    return response
    

@motor_crud_bp.route("/delete_motor_file", methods=["POST"])
def delete_motor_file():
    data = request.get_json()
    motor_file = data.get("motor_file")

    response = web_crud.delete_file(f"rov_config/motors/{motor_file}")
    return response
    

@motor_crud_bp.route("/merge_motor_files", methods=["POST"])
def merge_motor_files():
    data = request.get_json()
    motor_files = data.get("motor_files")
    merged_file = data.get("merged_file")

    try:
        merged_motors = []
        for motor_file in motor_files:
            motor_path = web_crud.resolve_file_path(
                f"rov_config/motors/{motor_file}")
            with open(motor_path, "r") as file:
                content = json.load(file)
                merged_motors.extend(content.get("motors", []))

        # throw error if duplicates are found
        motor_names = [motor.get("motor_name") for motor in merged_motors]
        if len(motor_names) != len(set(motor_names)):
            return jsonify({"success": False, "message": "Duplicate motor names found"})
        
        if len(merged_motors) == 0:
            return jsonify({"success": False, "message": "No motors to merge"})

        merged_path = web_crud.resolve_file_path(
            f"rov_config/motors/{merged_file}")
        merged_path.parent.mkdir(parents=True, exist_ok=True)
        with open(merged_path, "w") as file:
            json.dump({"motors": merged_motors}, file, indent=2)

        return jsonify({"success": True, "message": "Motor files merged successfully"}), 200

    except FileNotFoundError as exception:
        return jsonify({"success": False, "message": "One or more motor files not found"}), 500

    except PermissionError as exception:
        return jsonify({"success": False, "message": "Permission denied"}), 500

    except Exception as exception:
        return jsonify({"success": False, "message": f"Unexpected error: {str(exception)}"}), 500


@motor_crud_bp.route("/get_motor_files")
def get_motor_files():
    response = web_crud.get_directory_files("rov_config/motors")
    return response
    

@motor_crud_bp.route("/rename_motor_file", methods=["POST"])
def rename_motor_file():
    data = request.get_json()
    old_file = data.get("old_file")
    new_file = data.get("new_file")

    response = web_crud.rename_file(f"rov_config/motors/{old_file}", f"rov_config/motors/{new_file}")
    return response
