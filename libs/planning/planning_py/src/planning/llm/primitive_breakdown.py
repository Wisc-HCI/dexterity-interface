import json
import os
from typing import Any, Dict, List, Tuple, Optional
from planning.llm.llm import LLM

try:
    import yaml
except Exception:
    yaml = None


class PrimitiveBreakdown:
    def __init__(self, model:LLM, primitives_path:str):
        """
        Helper that formats inputs and asks the provided LLM for a primitive plan in JSON.

        Args:
            model (LLM): Generic LLM client instance (model-agnostic).
            primitives_path (str): Path to primitives catalog (.json or .yaml).
        """
        
        self._model = model
        self._catalog = self._load_catalog(primitives_path)


    def _load_catalog(self, path:str) -> Dict[str, Any]:
        """
        Load primitives catalog from JSON or YAML.

        Args:
            path (str): File path.

        Returns:
            Dict[str, Any]: Catalog dictionary with "low_level_primitives" and
                "mid_level_primitives" lists.
        """
        
        if not os.path.exists(path):
            raise FileNotFoundError(f"Primitives file not found: {path}")

        if path.endswith(".json"):
            with open(path, "r", encoding="utf-8") as f:
                return json.load(f)

        if path.endswith((".yaml", ".yml")) and yaml is not None:
            with open(path, "r", encoding="utf-8") as f:
                return yaml.safe_load(f)

        raise ValueError("Unsupported catalog format. Use .json or .yaml")


    def _schema_ok(self, data: Dict[str, Any]) -> Tuple[bool, str]:
        """
        Lightweight schema validation.

        Args:
            data (Dict[str, Any]): Parsed plan.

        Returns:
            (bool, str): (valid, error_message_if_any)
        """
        
        if not isinstance(data, dict):
            return False, "Root is not an object"
        if "primitive_plan" not in data or not isinstance(data["primitive_plan"], list):
            return False, "Missing 'primitive_plan' list"

        for idx, step in enumerate(data["primitive_plan"]):
            if not isinstance(step, dict):
                return False, f"Step {idx} is not an object"
            if "primitive_name" not in step or not isinstance(step["primitive_name"], str):
                return False, f"Step {idx} missing 'primitive_name' string"
            if "parameters" in step and not isinstance(step["parameters"], dict):
                return False, f"Step {idx} 'parameters' must be object"
            if "low_level_primitive_ordering" in step:
                if not isinstance(step["low_level_primitive_ordering"], list):
                    return False, f"Step {idx} 'low_level_primitive_ordering' must be list"
        return True, ""


    def _prompt_template(self, task_nl: str, objects_in_scene: List[Dict[str, Any]]) -> str:
        """
        Build strict JSON instruction prompt.

        Args:
            task_nl (str): Natural language task.
            objects_in_scene (List[Dict]): Objects with name/description/position.

        Returns:
            str: Prompt string.
        """
        
        catalog_text = json.dumps(self._catalog, ensure_ascii=False)
        objs_text = json.dumps({"objects_in_scene": objects_in_scene}, ensure_ascii=False)

        return f"""
        You are a planning assistant. You must output a single JSON object that matches this schema:

        {{
        "primitive_plan": [
            {{
            "primitive_name": "string",
            "parameters": {{}},
            "low_level_primitive_ordering": [{{"primitive_name": "string", "parameters": {{}}}}]
            }}
        ]
        }}

        Rules:
        - Use ONLY primitives provided in the catalog.
        - Only include parameters that exist for that primitive.
        - Output JSON ONLY (no prose).

        Task:
        {task_nl}

        Objects in scene:
        {objs_text}

        Primitive catalog:
        {catalog_text}
        """


    def plan(self, task_nl: str, objects_in_scene: List[Dict[str, Any]]) -> Dict[str, Any]:
        """
        Ask the LLM for a primitive plan and validate its JSON.

        Args:
            task_nl (str): Natural language task.
            objects_in_scene (List[Dict]): Objects in scene.

        Returns:
            Dict[str, Any]: Validated primitive plan.
        """
        
        prompt = self._prompt_template(task_nl, objects_in_scene)
        raw = self._model.prompt_json(prompt)
        data = json.loads(raw)

        ok, err = self._schema_ok(data)
        if not ok:
            if isinstance(data, list):
                data = {"primitive_plan": data}
            else:
                data = {"primitive_plan": data.get("primitive_plan", [])}
            ok, err = self._schema_ok(data)
            if not ok:
                raise ValueError(f"Plan JSON failed schema: {err}")

        return data
    