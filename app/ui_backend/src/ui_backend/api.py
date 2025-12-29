from ui_backend.schemas import Task, Primitive, Execution
from ui_backend.utils.UIBridgeNode import UIBridgeNode, RosRunner
from ui_backend.utils.utils import store_json, get_latest_json
from ui_backend.utils.helpers import get_current_scene

from planning.llm.gpt import GPT
from planning.llm.primitive_breakdown import PrimitiveBreakdown

import json
import time
from pathlib import Path
from typing import List

from contextlib import asynccontextmanager
from fastapi import FastAPI, Query
from fastapi.middleware.cors import CORSMiddleware



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
    """
    Takes a natural-language task description and generates a 
    sequence of executable robot primitives based on the current 
    scene configuration. Also saves the plan as json.

    Args:
        data (Task): Request payload containing the high-level task
            description to be planned.

    Returns:
        List[Primitive]: (x,) A flattened list of low-level primitives
            representing the execution plan. 
            Example: [{'type': 'grasp', 'arm': 'left', pose: [0,0,0,0,0,0,1]}, ...]
    """

    task = data.task
    scene = app.state.scene
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


@app.post("/api/execute_plan", response_model=Execution)
def execute_plan(primitives: List[Primitive],
                 on_real: bool = Query(False, description="Execute on real robot instead of simulation")):
    """
    Executes a sequence of primitives on either the simulated or real robot.

    Args:
        data (List[Primitive]): (x,) List of primitive actions to execute.
        on_real (bool): If True, executes on the real robot. Defaults to False (simulation).
    Returns:
        (dict): Execution metadata. Example: {'success': True, 'executed_on': 'real'}
    """

    # Reset objects
    for obj in app.state.scene:
        app.state.bridge_node.move_object(obj["name"], obj["position"])


    primitive_plan = [step.model_dump() for step in primitives]
    app.state.bridge_node.trigger_primitives(primitive_plan,  on_real=on_real)
    store_json(primitive_plan, JSON_DIR)
    return {'success': True, 'executed_on': 'real' if on_real else 'sim'}



@app.get("/api/primitive_plan/id/{item_id}", response_model=List[Primitive])
def get_plan(item_id: str):
    """
    Retrieve a previously stored primitive plan by identifier.

    Args:
        item_id (str): Identifier of the stored primitive plan
            (filename without extension).

    Returns:
        List[Primitive]: (x,) A flattened list of low-level primitives
            representing the execution plan. 
            Example: [{'type': 'grasp', 'arm': 'left', pose: [0,0,0,0,0,0,1]}, ...]
    """
    file_path = JSON_DIR / f"{item_id}.json"
    
    if not file_path.exists():
        return {"error": "Not found"}
    
    return json.loads(file_path.read_text())


@app.get("/api/primitive_plan/latest", response_model=List[Primitive])
def get_latest_plan() -> List[dict]:
    """
    Retrieve the most recently stored primitive plan.
    
    List[Primitive]: (x,) A flattened list of low-level primitives
            representing the execution plan. 
            Example: [{'type': 'grasp', 'arm': 'left', pose: [0,0,0,0,0,0,1]}, ...]
    """
    return get_latest_json(JSON_DIR)


