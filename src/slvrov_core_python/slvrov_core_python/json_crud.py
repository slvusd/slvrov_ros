import json
from pathlib import Path


def save_to_json(data: dict | object, filename: str | Path, indent: int = 2, enforce_unique_key: bool = False, overwrite: bool = False) -> None:
    """
    Save data to JSON file. Data can be a dict or an object with a __dict__ method. 
    If the file already exists, it will be overwritten if overwrite is True, otherwise the new data will be merged with the existing data. 
    If enforce_unique_key is True, an error will be raised if any keys in the new data already exist in the existing data (only applicable if overwrite is False).
    """
    json_file = Path(filename) if isinstance(filename, str) else filename
    json_file.touch(exist_ok=True)
        
    json_dict = {}

    # Converts data to dict if it's an object with a __dict__ method. Raises error if it's neither.
    if isinstance(data, dict): json_dict.update(data)
    else: 
        if hasattr(data, "__dict__") and callable(getattr(data, "__dict__")):
            json_dict.update(data.__dict__())
        else:
            raise ValueError(f"Data of type {type(data)} is not JSON serializable. Please provide a dict or an object with a __dict__ method.")

    # Loads existing data
    if not overwrite and json_file.stat().st_size > 0:
        with json_file.open("r") as file:
            try:
                json_dict.update(json.load(file))
            except json.JSONDecodeError:
                raise ValueError(f"JSON file {filename} is not valid JSON.")
            
    # Checks for unique keys if enforce_unique_key is True
    if enforce_unique_key:
        for key in data:
            if key in json_dict:
                raise ValueError(f"Key {key} already exists in JSON file {filename}. Set overwrite=True to overwrite existing data.")
            
    # Saves data
    with json_file.open("w") as file:
        json.dump(json_dict, file, indent=indent)