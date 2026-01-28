
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

TASKS = ["Pick the red cube"]

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