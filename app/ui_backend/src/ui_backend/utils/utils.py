from pathlib import Path
import ulid
import json


def store_json(json_data:dict, dir:Path):
    """
    Stores JSON data to disk. Does not write a new file if 
    the data matches the most recent entry.

    Args:
        json_data (dict): The JSON-serializable data to store.
        dir (Path): The directory where JSON files are saved.

    Returns:
        (dict): Metadata for the stored file in the form: 
            {'id': ..., 'created_at': ...}
            Returns an empty dictionary if the data matches the latest stored JSON.
    """
    if get_latest_json(dir) == json_data:
        return {}
    
    key = str(ulid.new())  # Time-sortable unique ID
    file_path = dir / f"{key}.json"
    file_path.write_text(json.dumps(json_data, indent=2))
    return { 
        "id": key,
        "created_at": key[:10]  # ULID embeds timestamp
    }


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
        print(files)
        return []
    
    latest_file = files[-1]  # Newest because ULID is sortable
    return json.loads(latest_file.read_text())

