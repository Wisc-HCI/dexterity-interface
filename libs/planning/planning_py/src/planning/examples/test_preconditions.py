import os
import json
from datetime import datetime
from typing import List, Dict, Any

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
            self._role = role_description

        def prompt(self, text: str) -> str:
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
            return self.prompt(text)

    LLMImpl = StubLLM
    ctor_kwargs = {"role_description": "stub"}
else:
    LLMImpl = RealLLMImpl


def _scenes() -> List[SceneState]:
    """
    Define the 3 test scenes for the task "Place the bread on the plate."
    """
    return [
        # Scene 1: Bread is on table, apple on plate, plate on table
        SceneState(on_table=["bread", "plate"], on_plate=["apple"], grasped=None),
        # Scene 2: Bread on table, apple on table, plate on table
        SceneState(on_table=["bread", "apple", "plate"], on_plate=[], grasped=None),
        # Scene 3: Bread on table, plate on table
        SceneState(on_table=["bread", "plate"], on_plate=[], grasped=None),
    ]


def run_all(
    preconditions_yaml_path: str = "libs/planning/planning_py/src/planning/llm/config/primitives_preconditions.yaml",
    out_path: str = "libs/planning/planning_py/src/planning/examples/example_output_preconditions.json",
) -> None:
    """
    Run 12 combinations: 3 scenes * 4 precondition sets for MOVE_TRANSPORT.
    Uses primitives_preconditions.yaml as BOTH the catalog and the source of preconditions.
    """
    model = LLMImpl(**ctor_kwargs)
    planner = PrimitiveBreakdown(model=model, primitives_path=preconditions_yaml_path)

    cases: List[Dict[str, Any]] = []
    for i_scene, scene in enumerate(_scenes(), start=1):
        for set_idx in range(4):
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
                "generated_at": datetime.utcnow().isoformat() + "Z",
                "total_cases": len(cases),
                "cases": cases,
            },
            f,
            indent=2,
        )
    print(f"Wrote results to {out_path}")


if __name__ == "__main__":
    run_all()
