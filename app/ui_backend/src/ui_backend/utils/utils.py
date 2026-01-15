from pathlib import Path
import ulid
import json
from datetime import datetime




def store_json(json_data:dict, dir:Path):
    """
    Stores JSON data to disk. Adds 'id' field to json.
    # Does not write a new file if 
    # the data matches the most recent entry.

    Args:
        json_data (dict): The JSON-serializable data to store.
        dir (Path): The directory where JSON files are saved.

    Returns:
        (dict): Metadata for the stored file in the form: 
            {'id': ..., 'created_at': ...}
            Returns an empty dictionary if the data matches the latest stored JSON.
    """

    key = str(ulid.new())  # Time-sortable unique ID
    json_data['id'] = key
    file_path = dir / f"{key}.json"
    file_path.write_text(json.dumps(json_data, indent=2))
    return json_data


def get_latest_json(dir:Path):
    """
    Retrieves the most recently stored JSON file from a directory.

    Args:
        dir (Path): The directory containing JSON files.

    Returns:
        (dict | list): The parsed contents of the most recent JSON file.
            Returns an empty list if no JSON files are present.
    """

    files = sorted(dir.glob("*.json"))
    
    if not files:
        return []
    
    latest_file = files[-1]  # Newest because ULID is sortable
    return json.loads(latest_file.read_text())


def get_all_json(dir:Path):
    """
    Retrieves all the json in a direcotory as list
    Args:
        dir (Path): The directory containing JSON files.

    Returns:
        (list): The parsed contents of all the JSON files concacted.
            Returns an empty list if no JSON files are present.
    """

    files = sorted(dir.glob("*.json"))
    
    if not files:
        return []
    
    dict_list = [json.loads(f.read_text()) for f in files]
    return dict_list

def get_json(id:str, dir:Path):
    """
    Retrieves the JSON with the give id name.

    Args:
        id (str): Name of the file (without the .json)
        dir (Path): The directory containing JSON files.

    Returns:
        (dict | list): The parsed contents of the most recent JSON file.
            Returns an empty list if no JSON files are present.
    """

    file_path = dir / f"{id}.json"
    
    if not file_path.exists():
        return {"error": "Not found"}
    
    return json.loads(file_path.read_text())