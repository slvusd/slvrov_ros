import json
from pathlib import Path
from flask import jsonify


CONFIG_DIR_NAME = "rov_config"
CONFIG_DIR_ALIASES = {CONFIG_DIR_NAME, "rov_configs"}


def get_config_root():
    """Find the repository-level ROV config directory."""

    for parent in Path(__file__).resolve().parents:
        candidate = parent / CONFIG_DIR_NAME
        if candidate.is_dir():
            return candidate

    return Path.cwd() / CONFIG_DIR_NAME


def resolve_file_path(file_path):
    path = Path(file_path)
    if path.is_absolute():
        return path

    if path.parts and path.parts[0] in CONFIG_DIR_ALIASES:
        config_root = get_config_root().resolve()
        resolved_path = (config_root / Path(*path.parts[1:])).resolve()
        resolved_path.relative_to(config_root)
        return resolved_path

    return (Path.cwd() / path).resolve()


def load_json_object(object_key, file_path):
    try:
        resolved_path = resolve_file_path(file_path)
        with open(resolved_path, "r") as file:
            content = json.load(file)
        
        return jsonify({"success": True, "object": content.get(object_key)}), 200
        
    except FileNotFoundError as exception:
        return jsonify({"success": False, "message": "File not found"}), 500
    
    except PermissionError as exception:
        return jsonify({"success": False, "message": "Permission denied"}), 500
    
    except Exception as exception:
        return jsonify({"success": False, "message": str(exception)}), 500


def write_json_object(object_key, value, file_path):
    try:
        resolved_path = resolve_file_path(file_path)
        resolved_path.parent.mkdir(parents=True, exist_ok=True)
        with open(resolved_path, "w") as file:
            json.dump({object_key: value}, file, indent=2)

        return jsonify({"success": True, "message": "Object written successfully"}), 200

    except FileNotFoundError as exception:
        return jsonify({"success": False, "message": "File not found"}), 500

    except PermissionError as exception:
        return jsonify({"success": False, "message": "Permission denied"}), 500

    except Exception as exception:
        return jsonify({"success": False, "message": str(exception)}), 500


def load_json_element(element_key, file_path):
    return load_json_object(element_key, file_path)


def write_json_element(element_key, value, file_path):
    return write_json_object(element_key, value, file_path)


def delete_object_in_json_array(array_key, object_field_key, object_key, file_path):
    try:
        resolved_path = resolve_file_path(file_path)
        with open(resolved_path, "r") as file:
            content = json.load(file)
            object_array = content.get(array_key)

        updated_array = [obj for obj in object_array if obj.get(object_field_key) != object_key]  # deletes key
        if len(updated_array) == len(object_array):
            raise ValueError("Object does not exist in specified file")

        with open(resolved_path, "w") as file:
            content[array_key] = updated_array
            json.dump(content, file, indent=2)

        return jsonify({"success": True, "message": "Object deleted successfully"}), 200

    except ValueError as exception:
        return jsonify({"success": False, "message": "Object does not exist in specified file"}), 500

    except FileNotFoundError as exception:
        return jsonify({"success": False, "message": "File not found"}), 500
    
    except PermissionError as exception:
        return jsonify({"success": False, "message": "Permission denied"}), 500
    
    except Exception as exception:
        return jsonify({"success": False, "message": str(exception)}), 500
    

def update_object_in_json_array(array_key, object_field_key, object_key, updated_object, file_path):
    try:
        resolved_path = resolve_file_path(file_path)
        with open(resolved_path, "r") as file:
            content = json.load(file)
            object_array = content.get(array_key)

        updated_array = [obj for obj in object_array if obj.get(object_field_key) != object_key]  # deletes key
        if len(updated_array) == len(object_array):
            raise ValueError("Object does not exist in specified file")
        
        updated_array.append(updated_object)  # adds back key with updated fields
        content[array_key] = updated_array

        with open(resolved_path, "w") as file:
            json.dump(content, file, indent=2)

        return jsonify({"success": True, "message": "Object updated successfully"}), 200

    except ValueError as exception:
        return jsonify({"success": False, "message": "Object does not exist in specified file"}), 500

    except FileNotFoundError as exception:
        return jsonify({"success": False, "message": "File not found"}), 500
    
    except PermissionError as exception:
        return jsonify({"success": False, "message": "Permission denied"}), 500
    
    except Exception as exception:
        return jsonify({"success": False, "message": str(exception)}), 500
    

def rename_file(old_file_path, new_file_path):
    try:
        resolved_old_path = resolve_file_path(old_file_path)
        resolved_new_path = resolve_file_path(new_file_path)
        resolved_new_path.parent.mkdir(parents=True, exist_ok=True)
        resolved_old_path.rename(resolved_new_path)
        return jsonify({"success": True, "message": "File renamed successfully"}), 200

    except FileNotFoundError as exception:
        return jsonify({"success": False, "message": "File not found"}), 500

    except PermissionError as exception:
        return jsonify({"success": False, "message": "Permission denied"}), 500

    except Exception as exception:
        return jsonify({"success": False, "message": str(exception)}), 500
    

def delete_file(file_path):
    try:
        resolve_file_path(file_path).unlink()
        return jsonify({"success": True, "message": "File deleted successfully"}), 200

    except FileNotFoundError as exception:
        return jsonify({"success": False, "message": "File not found"}), 500

    except PermissionError as exception:
        return jsonify({"success": False, "message": "Permission denied"}), 500

    except Exception as exception:
        return jsonify({"success": False, "message": str(exception)}), 500
    

def get_directory_files(directory_path):
    try:
        resolved_path = resolve_file_path(directory_path)
        files = [file.name for file in resolved_path.glob("*") if file.is_file()]
        return jsonify({"success": True, "files": files}), 200

    except Exception as exception:
        return jsonify({"success": False, "message": str(exception)}), 500
