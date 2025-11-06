import os
import json
from datetime import datetime, timezone
from dataclasses import dataclass
from typing import List, Dict, Any, Optional, Tuple

from planning.llm.primitive_breakdown import PrimitiveBreakdown, SceneState

RealLLMImpl = None
ctor_kwargs = {}

try:
    from planning.llm.gpt import GPT as RealLLMImpl
    try:
        ctor_kwargs = {"role_description": "You output JSON only that matches the schema, no prose."}
        _ = RealLLMImpl(**ctor_kwargs)
    except TypeError:
        ctor_kwargs = {"system_role": "You output JSON only that matches the schema, no prose."}
        _ = RealLLMImpl(**ctor_kwargs)
except Exception:
    RealLLMImpl = None


if RealLLMImpl is None:
    from planning.llm.llm import LLM

    class StubLLM(LLM):
        """
        Minimal concrete LLM that satisfies the abstract interface and returns
        a valid plan JSON string. This lets us produce 12 outputs deterministically.
        """

        def __init__(self, role_description: str = ""):
            """
            Initialize a deterministic stub LLM.

            Args:
                role_description (str): Optional textual role; stored only for parity
                    with real implementations.
            """

            self._role = role_description


        def prompt(self, text: str) -> str:
            """
            Return a deterministic MOVE_TRANSPORT plan as a JSON string.

            Args:
                text (str): The input prompt. Ignored by the stub for determinism.

            Returns:
                str: JSON string containing a single-step primitive plan that conforms
                to the expected schema.
            """

            return json.dumps({
                "primitive_plan": [
                    {
                        "primitive_name": "MOVE_TRANSPORT",
                        "parameters": {},
                        "low_level_primitive_ordering": [
                            {"primitive_name": "REACH", "parameters": {"target_location": [0, 0, 0]}},
                            {"primitive_name": "GRASP", "parameters": {"gripper_location": [0, 0, 0], "gripper_orientation": [0, 0, 0]}},
                            {"primitive_name": "RELEASE_OPEN", "parameters": {}}
                        ]
                    }
                ]
            })


        def prompt_json(self, text: str) -> str:
            """
            Return a deterministic MOVE_TRANSPORT plan as a JSON string.

            Args:
                text (str): The input prompt. Ignored by the stub for determinism.

            Returns:
                str: Same JSON string as `prompt`, provided for interface parity with
                real LLMs that expose a JSON-specific call.
            """

            return self.prompt(text)

    LLMImpl = StubLLM
    ctor_kwargs = {"role_description": "stub"}
else:
    LLMImpl = RealLLMImpl


@dataclass
class Pose:
    """
    Simple 3D pose for tabletop objects (meters; z is up).

    Attributes:
        x (float): X position in meters.
        y (float): Y position in meters.
        z (float): Z position in meters (upward).
    """
    x: float
    y: float
    z: float


@dataclass
class SceneStateCoords:
    """
    Coordinate-based scene for printing and precondition prompting.

    Attributes:
        objects (Dict[str, Pose]): Mapping from object name to its Pose.
        grasped (Optional[str]): Name of currently grasped object; None if free.
        support_graph (List[Tuple[str,str]]): Each tuple is (top, bottom) to
            indicate stacking relations (e.g., ("apple", "plate")).
    """
    objects: Dict[str, Pose]
    grasped: Optional[str]
    support_graph: List[Tuple[str, str]]


def _scenes_coords() -> List[SceneStateCoords]:
    """
    Build three physically sensible scenes with coordinates/stacks for the task
    "Place the bread on the plate."

    Returns:
        List[SceneStateCoords]: (3,) scenes; each scene contains:
            - objects (Dict[str, Pose]): Named object poses on a tabletop.
            - support_graph (List[Tuple[str,str]]): Stack relations (top -> bottom).
            - grasped (Optional[str]): Currently grasped object if any.
    """
    plate_xy = (0.50, 0.30)
    table_z = 0.75
    dz = 0.04

    def pose(x: float, y: float, z: float) -> Pose:
        return Pose(x, y, z)

    # Scene 1: plate occupied by apple; bread on table
    scene1 = SceneStateCoords(
        objects={
            "plate": pose(*plate_xy, table_z),
            "bread": pose(0.40, 0.30, table_z),
            "apple": pose(*plate_xy, table_z + dz),
        },
        grasped=None,
        support_graph=[("apple", "plate")],
    )

    # Scene 2: plate clear; bread and apple on table
    scene2 = SceneStateCoords(
        objects={
            "plate": pose(*plate_xy, table_z),
            "bread": pose(0.42, 0.32, table_z),
            "apple": pose(0.60, 0.30, table_z),
        },
        grasped=None,
        support_graph=[],
    )

    # Scene 3: plate clear; bread on table
    scene3 = SceneStateCoords(
        objects={
            "plate": pose(*plate_xy, table_z),
            "bread": pose(0.41, 0.29, table_z),
        },
        grasped=None,
        support_graph=[],
    )

    return [scene1, scene2, scene3]


class TestPreconditionPlanner(PrimitiveBreakdown):
    """
    Test-only subclass that builds a precondition-aware prompt using coordinate
    scenes while keeping PrimitiveBreakdown generic.
    """

    def _prompt_with_preconds_coords(
        self,
        task_nl: str,
        scene: SceneStateCoords,
        primitive_name: str,
        primitive_description: str,
        predicate_text: Dict[str, str],
        active_predicates: List[str],
    ) -> str:
        """
        Build a strict JSON prompt that injects active preconditions and a
        coordinate scene description.

        Args:
            task_nl (str): Natural language task description.
            scene (SceneStateCoords): Coordinate scene with objects/stacking state.
            primitive_name (str): Primitive under test (e.g., "MOVE_TRANSPORT").
            primitive_description (str): Human-readable description of the primitive.
            predicate_text (Dict[str,str]): Map from predicate key -> human text.
            active_predicates (List[str]): (K,) active predicate keys to enforce.

        Returns:
            str: Fully-rendered prompt string for the LLM that includes the
            catalog, the coordinate scene, and the human-readable preconditions.
        """
        precond_lines = [
            f"- {predicate_text.get(k, k.replace('_', ' ').title())}"
            for k in active_predicates
        ]
        preconds_block = "\n".join(precond_lines) if precond_lines else "- (none specified)"

        obj_lines: List[str] = []
        for name, p in scene.objects.items():
            obj_lines.append(f"- {name}: (x={p.x:.2f}, y={p.y:.2f}, z={p.z:.2f})")
        if scene.support_graph:
            obj_lines.append(f"- support_graph (top -> bottom): {scene.support_graph}")
        obj_lines.append(f"- grasped: {scene.grasped if scene.grasped else 'none'}")
        scene_block = "\n".join(obj_lines)

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

        Scene (coordinates):
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
        scene: SceneStateCoords,
        preconditions_yaml_path: str,
        precondition_set_index: int = 0,
        primitive_name: str = "MOVE_TRANSPORT",
    ) -> Dict[str, Any]:
        """
        Ask the LLM for a primitive plan that respects a chosen precondition set.

        Args:
            task_nl (str): Natural language task.
            scene (SceneStateCoords): Coordinate-based scene.
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
        prompt = self._prompt_with_preconds_coords(
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


def run_all(
    preconditions_yaml_path: str = "libs/planning/planning_py/src/planning/llm/config/primitives_preconditions.yaml",
    out_path: str = "libs/planning/planning_py/src/planning/examples/example_output_preconditions.json",
) -> None:
    """
    Run all combinations for MOVE_TRANSPORT using three scenes and four precondition
    sets (3 × 4 = 12 cases). For each case, print the coordinate scene summary and
    active preconditions, then write a JSON summary file.

    Args:
        preconditions_yaml_path (str): Path to the preconditions YAML file that also
            serves as the primitive catalog.
        out_path (str): Destination file path for the aggregated JSON results.

    Side Effects:
        - Prints each scene’s object coordinates, stacking relations, grasped object,
          and active preconditions (keys + human-readable text).
        - Creates parent directories for `out_path` if needed.
        - Writes a UTF-8 JSON file containing all cases, with a UTC timestamp.

    Returns:
        None
    """
    
    model = LLMImpl(**ctor_kwargs)
    planner = TestPreconditionPlanner(model=model, primitives_path=preconditions_yaml_path)

    _, predicate_text, precond_sets = planner._load_move_transport_preconditions(
        preconditions_yaml_path
    )

    cases: List[Dict[str, Any]] = []
    for i_scene, scene in enumerate(_scenes_coords(), start=1):
        for set_idx in range(len(precond_sets)):

            print(f"\n=== Scene {i_scene} | Precondition Set {set_idx} ===")
            print("Objects:")
            for name, p in scene.objects.items():
                print(f"  - {name}: (x={p.x:.2f}, y={p.y:.2f}, z={p.z:.2f})")
            if scene.support_graph:
                print("  support_graph (top -> bottom):", scene.support_graph)
            print("  grasped:", scene.grasped if scene.grasped else "none")
            keys = precond_sets[set_idx]
            print("  active predicates:", keys)
            if keys:
                print("  active predicates (human):")
                for k in keys:
                    print(f"   • {predicate_text.get(k, k.replace('_', ' ').title())}")

            result = planner.plan_with_preconditions(
                task_nl="Place the bread on the plate.",
                scene=scene,
                preconditions_yaml_path=preconditions_yaml_path,
                precondition_set_index=set_idx,
            )
            cases.append({
                "scene": i_scene,
                "precondition_set": set_idx + 1,
                "model_output": result,
            })

    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    with open(out_path, "w", encoding="utf-8") as f:
        json.dump(
            {
                "primitive": "MOVE_TRANSPORT",
                "generated_at": datetime.now(timezone.utc).isoformat(),
                "total_cases": len(cases),
                "cases": cases,
            },
            f,
            indent=2,
        )
    print(f"Wrote results to {out_path}")


if __name__ == "__main__":
    run_all()
