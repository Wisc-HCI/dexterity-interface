"""
Example: generate failure diagnosis results using GPT VLM + primitives.yaml + sample image
# TODO: replace sample image with actual camera feed of the failure scene

Run:
  python -m planning.examples.failure_primitive_preference
"""

import json
import os
import numpy as np
from PIL import Image
from typing import Dict, List

from planning.vlm.gpt import GPT

from planning.vlm.failure_primitive_preference import FailureDiagnose

PRIMS_PATH = os.path.join(
    os.path.dirname(os.path.dirname(__file__)),
    #"..",
    "llm",
    "config",
    "primitives.yaml",
)

IMAGE_PATH = os.path.join(
    os.path.dirname(os.path.dirname(__file__)),
    "vlm",
    "config",
    "peanut-butter.png",

)


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
    gpt = GPT("You are a precise planner and problem solver that always returns valid JSON.")
    planner = FailureDiagnose(gpt, PRIMS_PATH)

    results: List[Dict] = []

    for example in _build_examples():
        task = example["task"]
        scene = example["scene"]
        image = np.array(Image.open(IMAGE_PATH).convert("RGB"))
        diagnosis = planner.diagnose(task, image, scene)

        print("\nTask:", task)
        print(json.dumps(diagnosis, indent=2))

        results.append({
            "task": task,
            "scene": scene,
            "plan": diagnosis,
        })

    output_path = os.path.join(os.path.dirname(__file__), "example_diagnosis_output.json")
    with open(output_path, "w", encoding="utf-8") as f:
        json.dump(results, f, indent=2)

    print(f"\nSaved output to {output_path}")


if __name__ == "__main__":
    main()
