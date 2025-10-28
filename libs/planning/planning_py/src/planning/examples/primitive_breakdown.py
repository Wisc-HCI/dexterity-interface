"""
Example: generate primitive breakdowns using GPT + primitives.yaml

Run:
  python -m planning.examples.primitive_breakdown
"""

import json
import os
from typing import Dict, List

from planning.llm.gpt import GPT
from planning.llm.primitive_breakdown import PrimitiveBreakdown

PRIMS_PATH = os.path.join(
    os.path.dirname(__file__),
    "..",
    "llm",
    "config",
    "primitives.yaml",
)


def _test_scene_jar() -> List[Dict]:
    """
    Create a synthetic scene for a jar and its lid.

    Returns:
        List[Dict]: List of objects with name, description, and position [x,y,z] in meters.
    """
    return [
        {"name": "jar", "description": "Screwable jar", "position": [0.26, 0.17, 0.05]},
        {"name": "lid", "description": "Threaded lid", "position": [0.26, 0.17, 0.12]},
        {"name": "table", "description": "Flat surface", "position": [0.0, 0.0, 0.0]},
    ]


def _test_scene_bread() -> List[Dict]:
    """
    Create a synthetic scene for spreading peanut butter on bread.

    Returns:
        List[Dict]: List of objects with name, description, and position [x,y,z] in meters.
    """
    return [
        {"name": "knife", "description": "Butter knife", "position": [0.40, -0.17, 0.05]},
        {"name": "jar", "description": "Peanut butter", "position": [0.27, -0.09, 0.15]},
        {"name": "bread", "description": "Bread slice", "position": [0.36, 0.00, 0.09]},
    ]


def _build_examples() -> List[Dict]:
    """
    Build a small set of example tasks and scenes.

    Returns:
        List[Dict]: Each item has keys: 'task' (str) and 'scene' (List[Dict]).
    """
    return [
        {
            "task": "Open the jar by unscrewing the lid, then place the lid on the table.",
            "scene": _test_scene_jar(),
        },
        {
            "task": "Spread peanut butter on the bread using the knife.",
            "scene": _test_scene_bread(),
        },
    ]


def main() -> None:
    """
    Run example primitive breakdowns and print validated JSON outputs.

    Args:
        None

    Returns:
        None
    """
    gpt = GPT("You are a precise planner that always returns valid JSON.")
    planner = PrimitiveBreakdown(gpt, PRIMS_PATH)

    results: List[Dict] = []

    for example in _build_examples():
        task = example["task"]
        scene = example["scene"]
        plan = planner.plan(task, scene)

        print("\nTask:", task)
        print(json.dumps(plan, indent=2))

        results.append({
            "task": task,
            "scene": scene,
            "plan": plan,
        })

    output_path = os.path.join(os.path.dirname(__file__), "example_output.json")
    with open(output_path, "w", encoding="utf-8") as f:
        json.dump(results, f, indent=2)

    print(f"\nSaved output to {output_path}")


if __name__ == "__main__":
    main()
