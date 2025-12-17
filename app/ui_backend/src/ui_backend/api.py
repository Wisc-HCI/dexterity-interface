from ui_backend.UIBridgeNode import UIBridgeNode, RosRunner
from contextlib import asynccontextmanager

from pathlib import Path
from typing import Optional, List

from planning.llm.gpt import GPT
from planning.llm.primitive_breakdown import PrimitiveBreakdown

from fastapi import FastAPI, Query
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel

   
import ulid
import json


runner = RosRunner()
bridge_node = UIBridgeNode()

@asynccontextmanager
async def lifespan(app: FastAPI):
    runner.start(bridge_node)
    yield
    runner.stop()

app = FastAPI(lifespan=lifespan)

# Allow your frontend to call the API during dev
app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:3000",],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


# Task Breakdown (TODO: MAKE MODULAR)
root_dir = Path(__file__).resolve().parents[4]
prims_path = str(root_dir/"libs"/"planning"/"planning_py"/"src"/"planning"/"llm"/"config"/"core_primitives.yaml")

gpt = GPT("You are a precise planner that always returns valid JSON. Note: downward gripper is [qx, qy, qz, qw] = [ 0.707, 0.707, 0.0, 0.0]")
planner = PrimitiveBreakdown(gpt, prims_path)

JSON_DIR = Path(__file__).resolve().parent / "json_primitives"
JSON_DIR.mkdir(exist_ok=True)
print("JSON DIR", JSON_DIR)

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


def get_current_scene():
    # TODO
    # return [
    #     {"name": "cube", "description": "blue block", "position": [0.3, 0.2, 0.95, 0.0, 0.0, 0.0, 1.0]},
    #     {"name": "bowl", "description": "bowl", "position": [ 0.3, -0.2, 0.95, 0.0, 0.0, 0.0, 1.0]},
    # ]

    # TODO: PASS grasp pose better
    return [
        {"name": "cup", "description": "Small cup. Height: 0.08 (m). Use this grasp pose: [0.2, 0.11, 1, 0, -0.818, 0.574, 0]", "position": [0.2, 0.1, 0.95, 0.0, 0.0, 0.0, 1.0]},
        {"name": "bowl", "description": "Bowl. Height: 0.0476, Width:0.18 (m). Pour location: [0.2, -0.2, 1.15, 0, -0.788, -0.614, 0]", "position": [ 0.2, -0.2, 0.95, 0.0, 0.0, 0.0, 1.0]},
    ]







SCENE = get_current_scene()
import time
time.sleep(1)
for obj in SCENE:
    bridge_node.spawn_object(obj["name"], obj["position"])



class Task(BaseModel):
    task: str

class Primitive(BaseModel):
    type: str
    arm: Optional[str] = None
    pose: Optional[list[float]] = None
    
# @app.get("/api/test", )
# def get_test():
#     """
#     TODO
#     """
#     prims = [
#         {'type': 'home'},
#         {'type': 'move_to_pose', 'arm': 'left', 'pose': [-0.2, 0.2, 0.2, 0.707, 0.707, 0.0, 0.0 ]},
#         {'type': 'envelop_grasp', 'arm': 'left'},
#         {'type': 'release', 'arm': 'left'},
#         {'type': 'envelop_grasp', 'arm': 'right'},
#         {'type': 'release', 'arm': 'right'}
#     ]
#     bridge_node.trigger_primitives(prims)
    
#     return {"data": "test successful"}


@app.post("/api/primitive_plan", response_model=List[Primitive])
def primitive_plan(data: Task):
    """TODO"""

    task = data.task
    scene = SCENE
    # TODO: ADD CURRENT PLAN HERE
    plan = planner.plan(task, scene)

    print("PLAN", plan)
   

    #TODO: CLEAN
    remapped = []
    for prim in plan.get("primitive_plan", []):
        low_level = prim.get("low_level_primitive_ordering")

        # Flatten low level
        if low_level:
            for step in low_level:
                new_step = {
                    "type": step["primitive_name"],
                    **(step.get("parameters") or {})
                }
                remapped.append(new_step)

        # Else use primitive itself
        else:
            new_step = {
                "type": prim["primitive_name"],
                **(prim.get("parameters") or {})
            }
            remapped.append(new_step)

    print("PLAN", remapped)
    store_json(remapped, JSON_DIR)


    return remapped


@app.post("/api/execute_plan")
def execute_plan(primitives: List[Primitive],
                 on_real: bool = Query(False, description="Execute on real robot instead of simulation")):
    """
    Executes a sequence of primitives on either the simulated or real robot.

    Args:
        data (List[Primitive]): List of primitive actions to execute.
        on_real (bool): If True, executes on the real robot. Defaults to False (simulation).
    """

    # Reset objects
    for obj in SCENE:
        bridge_node.move_object(obj["name"], obj["position"])


    primitive_plan = [step.model_dump() for step in primitives]
    bridge_node.trigger_primitives(primitive_plan,  on_real=on_real)
    store_json(primitive_plan, JSON_DIR)
    return {'success': True, 'executed_on': 'real' if on_real else 'sim'}



@app.get("/api/primitive_plan/id/{item_id}")
def get_json(item_id: str):
    """TODO"""
    file_path = JSON_DIR / f"{item_id}.json"
    
    if not file_path.exists():
        return {"error": "Not found"}
    
    return json.loads(file_path.read_text())


@app.get("/api/primitive_plan/latest")
def get_latest() -> List[dict]:
    """TODO"""
    return get_latest_json(JSON_DIR)


