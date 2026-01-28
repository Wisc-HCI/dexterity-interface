
"""
Script to generate a bunch of plans all at once (same thing you can do on interface)
Setup:
pip install -e libs/planning/planning_py/
pip install -e app/ui_backend

Running:
python3 -m ui_backend.generate_plans
"""

from planning.llm.gpt import GPT
from planning.llm.claude import Claude
from planning.llm.primitive_breakdown import PrimitiveBreakdown
from ui_backend.utils.helpers import get_current_scene
from ui_backend.utils.utils import store_json


import json
from pathlib import Path
from typing import List, Optional



AGENT = GPT
# AGENT = Claude

TASKS = [
    "Pick the red cube.",
    "Pick the green cube.",
    "Pick the blue cube.",
    "Move the red cube.",
    "Move the blue cube.",
    "Move the green cube.",
    "Move the blue cube left.",
    "Move the red cube left.",
    "Move the green cube left.",

    "Pick the red cube, then pick the green cube.",
    "Pick the green cube, then pick the blue cube.",
    "Pick the blue cube, then pick the red cube.",
    "Move the red cube, then pick the blue cube.",
    "Move the blue cube, then pick the green cube.",
    "Move the green cube, then pick the red cube.",
    "Move the blue cube left, then pick the red cube.",
    "Move the red cube left, then lift the green cube.",
    "Move the green cube left, then lift the blue cube.",

    "Pick the red cube, then pick the green cube, then pick the blue cube.",
    "Pick the green cube, then pick the blue cube, then pick the red cube.",
    "Pick the blue cube, then pick the red cube, then pick the green cube.",
    "Move the red cube right then left, then pick the blue cube.",
    "Move the blue cube right then left, then pick the green cube.",
    "Move the green cube right then left, then pick the red cube.",
    "Move the blue cube left then right, then pick the red cube.",
    "Move the red cube left then right, then lift the green cube.",
    "Move the green cube left then right, then lift the blue cube."
]

NUM_TRIALS = 3

JSON_DIR = Path(__file__).resolve().parent / "json_primitives"
PRIMS_PATH = str(Path(__file__).resolve().parents[4]/"libs"/"planning"/"planning_py"/"src"/"planning"/"llm"/"config"/"primitives.yaml")


agent = AGENT("You are a precise planner that always returns valid JSON. " \
    "Notes: Downward gripper is [qx, qy, qz, qw] = [ 0.707,0.707,0.0,0.0]" \
    "And right is positive x, forward is positive x, up is positive y." \
    "The left robot is at [-0.5,-0.09,0.9] and the right  is at [0.5,-0.09,0.9] ([x,y,z] in m)." \
    "Always home at the beginning of the plan.",
                    save_history=False)
planner = PrimitiveBreakdown(agent, PRIMS_PATH)
scene = get_current_scene()



for task in TASKS:
    for i in range(NUM_TRIALS):
        prim_plan = planner.plan(task, scene).get("primitive_plan", [])

        # Add in so comparison works once sent back.
        for prim in prim_plan:
            prim['core_primitives'] = None

        data_to_store = {
            'id': None,  # Added in store_json
            'revision_of': None,
            'task_prompt': task,
            'primitive_plan': prim_plan
        }
        stored_data = store_json(data_to_store, JSON_DIR)