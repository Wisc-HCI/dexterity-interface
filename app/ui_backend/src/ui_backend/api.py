from ui_backend.schemas import Task, Primitive, Execution
from ui_backend.utils.UIBridgeNode import UIBridgeNode, RosRunner
from ui_backend.utils.utils import store_json, get_latest_json
from ui_backend.utils.helpers import get_current_scene

from primitives_ros.utils.create_high_level_prims import parse_prim_plan, flatten_hierarchical_prims
from planning.llm.gpt import GPT
from planning.llm.primitive_breakdown import PrimitiveBreakdown

import json
from pathlib import Path
from typing import List, Optional

from contextlib import asynccontextmanager
from fastapi import FastAPI, Query
from fastapi.middleware.cors import CORSMiddleware



########################################################
####################### CONSTANTS #######################
JSON_DIR = Path(__file__).resolve().parent / "json_primitives"
PRIMS_PATH = str(Path(__file__).resolve().parents[4]/"libs"/"planning"/"planning_py"/"src"/"planning"/"llm"/"config"/"primitives.yaml")


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
    app.state.flat_to_hierach_idx_map = None
    app.state.flat_start_idx = None

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
    allow_origins=["http://localhost:3000", "http://127.0.0.1:3000"],
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
        app.state.bridge_node.spawn_object(obj["name"], obj["pose"])

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
        List[Primitive]: (x,) The primitive plan build from core primitives
            Example: [{'name': 'envelop_grasp', parameters: {'arm': 'left', pose: [0,0,0,0,0,0,1]}, core_primitives: {...} }, ...]
    """

    task = data.task
    scene = app.state.scene
    plan = app.state.planner.plan(task, scene)

    high_level_plan = plan.get("primitive_plan", [])
    
    parsed_out_plan = parse_prim_plan(high_level_plan)

    store_json(parsed_out_plan, JSON_DIR)

    return parsed_out_plan


@app.post("/api/execute_plan", response_model=Execution)
def execute_plan(primitives: List[Primitive],
                 on_real: bool = Query(False, description="Execute on real robot instead of simulation"),
                 start_index: Optional[List[int]] = Query(None,description="Optional hierarchical start index as JSON array, e.g. [0,1,2]")
                 ):
    """
    Executes a sequence of primitives on either the simulated or real robot.

    Args:
        data (List[Primitive]): (x,) List of primitive actions to execute.
        on_real (bool): If True, executes on the real robot. Defaults to False (simulation).
        start_index (List[int]): Hierarchical start index of where to start plan execution.
    Returns:
        (dict): Execution metadata. Example: {'success': True, 'executed_on': 'real'}
    """

    # Reset objects
    # TODO: HANDLE OBJECTS MORE COMPLEXlEY
    app.state.bridge_node.move_objects(app.state.scene)


    primitive_plan = [step.model_dump() for step in primitives]
    flattened_plan, flat_to_hierach_idx_map, hierach_to_flat_idx_map = flatten_hierarchical_prims(primitive_plan)

    if start_index is not None:
        flat_start_idx = hierach_to_flat_idx_map[tuple(start_index)]
        flattened_plan = flattened_plan[flat_start_idx:]
        app.state.flat_start_idx = flat_start_idx
    else:
        app.state.flat_start_idx = None

    app.state.bridge_node.trigger_primitives(flattened_plan, on_real=on_real)



    store_json(primitive_plan, JSON_DIR)
    app.state.flat_to_hierach_idx_map = flat_to_hierach_idx_map

    return {'success': True, 'executed_on': 'real' if on_real else 'sim'}



@app.get("/api/primitive_plan/id/{item_id}", response_model=List[Primitive])
def get_plan(item_id: str):
    """
    Retrieve a previously stored primitive plan by identifier.

    Args:
        item_id (str): Identifier of the stored primitive plan
            (filename without extension).

    Returns:
        (List[Primitive]): (x,) A list of  primitives
            representing the execution plan. 
            Example: [{'name': 'envelop_grasp', parameters: {'arm': 'left', pose: [0,0,0,0,0,0,1]}, core_primitives: {...} }, ...]
    """
    file_path = JSON_DIR / f"{item_id}.json"
    
    if not file_path.exists():
        return {"error": "Not found"}
    
    return json.loads(file_path.read_text())


@app.get("/api/primitive_plan/latest", response_model=List[Primitive])
def get_latest_plan() -> List[Primitive]:
    """
    Retrieve the most recently stored primitive plan.
    Returns:
        (List[Primitive]): (x,) A list of primitives
                representing the execution plan. 
                Example: [{'name': 'envelop_grasp', parameters: {'arm': 'left', pose: [0,0,0,0,0,0,1]}, core_primitives: {...} }, ...]
    """
    return get_latest_json(JSON_DIR)


@app.post("/api/primitive", response_model=Primitive)
def update_primitive(primitive: Primitive) -> Primitive:
    """
    Given new parameters, regenerates the prims for a given high-level prim.
    Args:
        primitive (Primitive): The high-level primitive in the form of:
            {'name': 'envelop_grasp', parameters: {'arm': 'left', pose: [0,0,0,0,0,0,1]}, core_primitives: {...} }

    Returns:
        (Primitive): The regenerated high-level primitive in the form of
            {'name': 'envelop_grasp', parameters: {'arm': 'left', pose: [0,0,0,0,0,0,1]}, core_primitives: {...} }
    """
    regenerated_prim = parse_prim_plan([primitive.model_dump()])[0]
    return regenerated_prim


@app.get("/api/executing_primitive_idx", response_model=Optional[List[int]])
def get_current_executing_primitive() -> Optional[List[int]]:
    """
    Gets the current executing primitive index.
  
    Returns:
        (list[int]): The index of the currently executing primitive in the form of [first-level-idx,sec-level-idx,...]
            based on the primitive hierarchy from the most recently posted plan to execute.
    """
    flat_idx = app.state.bridge_node.get_curr_executing_idx()

    if flat_idx is None:
        return None
    
    if app.state.flat_start_idx is not None:
        flat_idx +=  app.state.flat_start_idx
        
    hierarchical_idx = app.state.flat_to_hierach_idx_map[flat_idx]

    return hierarchical_idx



@app.post("/api/primitive_plan/cancel")
def stop_plan_execution():
    """
    Cancels the currently executing primitive plan.
    """
    app.state.bridge_node.cancel_primitives_goal()
    return {'status': 'cancel message sent'}