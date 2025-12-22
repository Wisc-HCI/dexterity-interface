from ui_backend.utils.UIBridgeNode import UIBridgeNode, RosRunner
from ui_backend.utils.utils import store_json, get_latest_json
from ui_backend.utils.helpers import get_current_scene

from planning.llm.gpt import GPT
from planning.llm.primitive_breakdown import PrimitiveBreakdown

import json
import time
from pathlib import Path
from typing import Optional, List

from contextlib import asynccontextmanager
from fastapi import FastAPI, Query
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel


########################################################
####################### CONSTANTS #######################
JSON_DIR = Path(__file__).resolve().parent / "json_primitives"
PRIMS_PATH = str(Path(__file__).resolve().parents[4]/"libs"/"planning"/"planning_py"/"src"/"planning"/"llm"/"config"/"core_primitives.yaml")


########################################################
####################### Lifespan #######################
@asynccontextmanager
async def lifespan(app: FastAPI):
    """
    Handles starting and stopping of app

    Args:
        app (FastAPI): app object
    """
    JSON_DIR.mkdir(exist_ok=True)

    # "Global" variables
    app.state.runner = RosRunner()
    app.state.bridge_node = UIBridgeNode() # Must run RosRunner first for rclpy.init()
    
    app.state.gpt = GPT("You are a precise planner that always returns valid JSON. Note: downward gripper is [qx, qy, qz, qw] = [ 0.707, 0.707, 0.0, 0.0]")
    app.state.planner = PrimitiveBreakdown(app.state.gpt, PRIMS_PATH)
    app.state.scene = get_current_scene()

    # Start ROS Node
    app.state.runner.start(app.state.bridge_node)



    # When app closes
    yield
    app.state.runner.stop()


########################################################
##################### App Creation #####################
app = FastAPI(lifespan=lifespan)

# Allow your frontend to call the API during dev
app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:3000",],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


########################################################
######################## Schemas #######################

class Task(BaseModel):
    task: str

class Primitive(BaseModel):
    type: str
    arm: Optional[str] = None
    pose: Optional[list[float]] = None
    


########################################################
######################## Routes #######################

@app.post("/api/spawn_objects")
def spawn_objects():
    """
    Call to initialize objects in the scene
    """
    for obj in app.state.scene:
        app.state.bridge_node.spawn_object(obj["name"], obj["position"])

    return {'success': True}

@app.post("/api/primitive_plan", response_model=List[Primitive])
def primitive_plan(data: Task):
    """TODO"""

    task = data.task
    scene = app.state.scene
    # TODO: ADD CURRENT PLAN HERE
    plan = app.state.planner.plan(task, scene)

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
    for obj in app.state.scene:
        app.state.bridge_node.move_object(obj["name"], obj["position"])


    primitive_plan = [step.model_dump() for step in primitives]
    app.state.bridge_node.trigger_primitives(primitive_plan,  on_real=on_real)
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


