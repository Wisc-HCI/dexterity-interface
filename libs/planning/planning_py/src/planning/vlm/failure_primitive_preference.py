import json
import os
import numpy as np
from typing import Any, Dict, List, Tuple, Optional
from planning.vlm.vlm import VLM

try:
    import yaml
except Exception:
    yaml = None

class FailureDiagnose:
    def __init__(self, model:VLM, primitives_path:str):
        """
        Helper that formats inputs and asks the provided VLM for a primitive plan in JSON.

        Args:
            model (VLM): Generic VLM client instance (model-agnostic).
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
        if "failure_diagnosis" not in data or not isinstance(data["failure_diagnosis"], list):
            return False, "Missing 'failure_diagnosis' list"

        for idx, step in enumerate(data["failure_diagnosis"]):
            if not isinstance(step, dict):
                return False, f"Step {idx} is not an object"
            if "primitive" not in step or not isinstance(step["primitive"], str):
                return False, f"Step {idx} missing 'primitive' string"
            if "assumed user preference/default setting causing failure" in step and not isinstance(step["assumed user preference/default setting causing failure"], str):
                return False, f"Step {idx} 'assumed user preference/default setting causing failure' must be string"
        return True, ""


    def _prompt_template(self, task_nl: str, objects_in_scene: List[Dict[str, Any]], prior_plan: List[dict]=None) -> str:
        """
        Build strict JSON instruction prompt.

        Args:
            task_nl (str): Natural language task.
            objects_in_scene (List[Dict]): Objects with name/description/pose.
            prior_plan(List[dict]): [Optional] Prior plan to pass as context.

        Returns:
            str: Prompt string.
        """
        
        catalog_text = json.dumps(self._catalog, ensure_ascii=False)
        objs_text = json.dumps({"objects_in_scene": objects_in_scene}, ensure_ascii=False)

        return f"""
        You are a planning assistant. You must identify what went wrong based on the image and output a single JSON object that matches this schema:

        {{
        "failure_diagnosis": [
            {{
            "primitive": "string",
            "assumed user preference/default setting causing failure": "string",
            }}
        ]
        }}

        Rules:
        - Use ONLY primitives provided in the catalog.
        - Only include parameters that exist for that primitive.
        - Output JSON ONLY (no prose).
        - If prior plan is provided, use it as context for the task.

        Task:
        {task_nl}

        Objects in scene:
        {objs_text}

        Prior Plan:
        {prior_plan}

        Primitive catalog:
        {catalog_text}
        """


    def diagnose(self, task_nl: str, image_np: np.ndarray, objects_in_scene: List[Dict[str, Any]], prior_plan: List[dict]=None) -> Dict[str, Any]:
        """
        Ask the VLM for a failure diagnosis and validate its JSON.

        Args:
            task_nl (str): Natural language task.
            image_np (np.ndarray): Image of the failure scene.
            objects_in_scene (List[Dict]): Objects in scene.
            prior_plan(List[dict]): [Optional] Prior plan to pass as context.


        Returns:
            Dict[str, Any]: Validated failure diagnosis.
        """
        
        prompt_text = self._prompt_template(task_nl, objects_in_scene, prior_plan)

        raw = self._model._prompt_with_image(prompt_text, image_np)

        raw = raw.strip()

        # Remove ```json ... ``` or ``` ... ```
        if raw.startswith("```"):
            raw = raw.strip("`")
            if raw.lstrip().startswith("json"):
                raw = raw.lstrip()[4:]  # remove leading 'json'
            raw = raw.strip()

        data = json.loads(raw)

        ok, err = self._schema_ok(data)
        if not ok:
            if isinstance(data, list):
                data = {"failure_diagnosis": data}
            else:
                data = {"failure_diagnosis": data.get("failure_diagnosis", [])}
            ok, err = self._schema_ok(data)
            if not ok:
                raise ValueError(f"Plan JSON failed schema: {err}")

        return data
    