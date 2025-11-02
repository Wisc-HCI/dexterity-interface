import json
import os
from typing import Any, Dict, List, Tuple, Optional
from planning.llm.llm import LLM

try:
    import yaml
except Exception:
    yaml = None

class SceneState:
    """
    Represents a simplified world state for precondition tests.

    Args:
        on_table (List[str]): (N,) names of objects on the table.
        on_plate (List[str]): (M,) names of objects already on the plate.
        grasped (Optional[str]): Name of object currently grasped, or None.

    Returns:
        SceneState: Snapshot used to build LLM prompt.
    """
    def __init__(self, on_table: List[str], on_plate: List[str], grasped: Optional[str]):
        self._on_table = on_table
        self._on_plate = on_plate
        self._grasped = grasped

    # getters (private attributes per convention)
    @property
    def on_table(self) -> List[str]:
        return self._on_table

    @property
    def on_plate(self) -> List[str]:
        return self._on_plate

    @property
    def grasped(self) -> Optional[str]:
        return self._grasped

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
    

    def _load_move_transport_preconditions(
        self, path: str
    ) -> Tuple[str, Dict[str, str], List[List[str]]]:
        """
        Load MOVE_TRANSPORT preconditions from the current YAML schema:

        - llm_predicates: {PREDICATE_KEY: "human readable text", ...}
        - low_level_primitives: [ { name: MOVE_TRANSPORT, precondition_sets: [...] , ...}, ... ]

        Args:
            path (str): Path to primitives_preconditions.yaml.

        Returns:
            Tuple[str, Dict[str, str], List[List[str]]]:
                description, predicate_text, precondition_sets
        """

        if yaml is None:
            raise ImportError("pyyaml is required to read YAML preconditions.")
        if not os.path.exists(path):
            raise FileNotFoundError(f"Preconditions file not found: {path}")

        with open(path, "r", encoding="utf-8") as f:
            cfg = yaml.safe_load(f)

        predicate_text: Dict[str, str] = cfg.get("llm_predicates", {})

        low_level = cfg.get("low_level_primitives", [])
        mt = None
        for p in low_level:
            if isinstance(p, dict) and p.get("name") == "MOVE_TRANSPORT":
                mt = p
                break

        if mt is None:
            raise ValueError("MOVE_TRANSPORT not found in preconditions YAML")

        description: str = mt.get("description", "MOVE_TRANSPORT")

        raw_sets = mt.get("precondition_sets", None)
        if raw_sets is None:
            legacy = mt.get("pre-condition", "")
            raw_sets = [[legacy]] if isinstance(legacy, str) and legacy.strip() else [[]]

        norm_sets: List[List[str]] = []
        for s in raw_sets:
            if isinstance(s, list):
                norm_sets.append(s)
            elif isinstance(s, str):
                norm_sets.append([s])
            else:
                raise ValueError("Each precondition set must be a list or string.")

        return description, predicate_text, norm_sets
    

    def _prompt_with_preconds(
        self,
        task_nl: str,
        scene: SceneState,
        primitive_name: str,
        primitive_description: str,
        predicate_text: Dict[str, str],
        active_predicates: List[str],
    ) -> str:
        """
        Build a strict JSON prompt that injects the active preconditions and scene state.

        Args:
            task_nl (str): Natural language task (e.g., "Place the bread on the plate.").
            scene (SceneState): Current simplified scene.
            primitive_name (str): Primitive under test (e.g., MOVE_TRANSPORT).
            primitive_description (str): Human-readable primitive description.
            predicate_text (Dict[str, str]): Mapping predicate -> natural language.
            active_predicates (List[str]): Preconditions to enforce.

        Returns:
            str: Prompt string for the LLM.
        """

        precond_lines: List[str] = []
        for pred in active_predicates:
            human = predicate_text.get(pred, pred.replace("_", " ").title())
            precond_lines.append(f"- {human}")
        preconds_block = "\n".join(precond_lines) if precond_lines else "- (none specified)"

        scene_lines = [
            f"- On table: {', '.join(scene.on_table) if scene.on_table else 'none'}",
            f"- On plate: {', '.join(scene.on_plate) if scene.on_plate else 'none'}",
            f"- Grasped: {scene.grasped if scene.grasped else 'none'}",
        ]
        scene_block = "\n".join(scene_lines)

        catalog_text = json.dumps(self._catalog, ensure_ascii=False)

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

        Primitive under test:
        - Name: {primitive_name}
        - Description: {primitive_description}

        Scene state:
        {scene_block}

        Preconditions the plan MUST respect:
        {preconds_block}

        If a precondition is violated by the scene, propose minimal corrective actions BEFORE moving/placing.

        Primitive catalog:
        {catalog_text}
        """
    

    def plan_with_preconditions(
    self,
    task_nl: str,
    scene: SceneState,
    preconditions_yaml_path: str,
    precondition_set_index: int = 0,
    primitive_name: str = "MOVE_TRANSPORT",
    ) -> Dict[str, Any]:
        """
        Ask the LLM for a primitive plan that respects a chosen precondition set.

        Args:
            task_nl (str): Natural language task (e.g., "Place the bread on the plate.").
            scene (SceneState): Simplified scene state for the precondition tests.
            preconditions_yaml_path (str): Path to primitives_preconditions.yaml.
            precondition_set_index (int): Which set to use (0-based index).
            primitive_name (str): Primitive to test (default "MOVE_TRANSPORT").

        Returns:
            Dict[str, Any]: Validated primitive plan.
        """

        description, predicate_text, precond_sets = self._load_move_transport_preconditions(
            preconditions_yaml_path
        )
        if not (0 <= precondition_set_index < len(precond_sets)):
            raise IndexError(
                f"precondition_set_index out of range (got {precondition_set_index}, "
                f"available 0..{len(precond_sets)-1})."
            )

        active_preds = precond_sets[precondition_set_index]
        prompt = self._prompt_with_preconds(
            task_nl=task_nl,
            scene=scene,
            primitive_name=primitive_name,
            primitive_description=description,
            predicate_text=predicate_text,
            active_predicates=active_preds,
        )

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
