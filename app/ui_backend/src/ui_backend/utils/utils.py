from pathlib import Path
import ulid
import json


def store_json(json_data:dict, dir:Path):
    # Don't save if same as as prior
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
    """TODO"""
    
    files = sorted(dir.glob("*.json"))
    
    if not files:
        print(files)
        return []
    
    latest_file = files[-1]  # Newest because ULID is sortable
    return json.loads(latest_file.read_text())

