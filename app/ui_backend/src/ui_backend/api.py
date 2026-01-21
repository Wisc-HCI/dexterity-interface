from ui_backend.schemas import Primitive, Execution, Plan, NewPlan, RevisedPlan
from ui_backend.utils.UIBridgeNode import UIBridgeNode, RosRunner
from ui_backend.utils.utils import store_json, get_latest_json, get_json, get_all_json

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
    
    app.state.gpt = GPT("You are a precise planner that always returns valid JSON. " \
        "Notes: Downward gripper is [qx, qy, qz, qw] = [ 0.707,0.707,0.0,0.0]" \
        "And right is positive x, forward is positive x, up is positive y." \
        "The left robot is at [-0.5,-0.09,0.9] and the right  is at [0.5,-0.09,0.9] ([x,y,z] in m)",
                        save_history=False)
    app.state.planner = PrimitiveBreakdown(app.state.gpt, PRIMS_PATH)
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
    
    app.state.bridge_node.spawn_objects()

    return {'success': True}



@app.post("/api/primitive_plan", response_model=Plan)
def primitive_plan(req: NewPlan):
    """
    Takes a natural-language task description and generates a 
    sequence of executable robot primitives based on the current 
    scene configuration. Also saves the plan as json.

    Args:
        req (NewPlan): Request payload containing:
            - task_prompt (str): Natural-language description of the task.
            - revision_of (str): (Optional) Identifier of the plan being revised.
                (usually id of most recent plan)

    Returns:
        (Plan): The newly stored plan object, including:
            - id: Unique identifier assigned during storage.
            - revision_of: None (first plan in the session).
            - task_prompt: The prompt used to generate the plan.
            - primitive_plan: Parsed list of executable primitives.
    """
    revision_of = req.revision_of
    task_prompt = req.task_prompt

    prior_version = None

    if revision_of:
        prior_version = get_json(revision_of, JSON_DIR)

    scene = app.state.bridge_node.get_scene()
    plan = app.state.planner.plan(task_prompt, scene, prior_version)
    high_level_plan = plan.get("primitive_plan", [])
    parsed_out_plan = parse_prim_plan(high_level_plan)
    data_to_store = {
        'id': None,  # Added in store_json
        'revision_of': revision_of,
        'task_prompt': task_prompt,
        'primitive_plan': parsed_out_plan
    }
    stored_data = store_json(data_to_store, JSON_DIR)

    
    return stored_data
        



@app.post("/api/primitive_plan_revision", response_model=Plan)
def primitive_plan_revision(req: RevisedPlan):
    """
    Save a revised version of an existing primitive plan.

    Args:
        req (RevisedPlan): Request payload containing:
            - revision_of (str): Identifier of the plan being revised.
            - task_prompt (str): Prompt or description associated with
              the revision.
            - primitive_plan (List[Primitive]): The revised primitive plan.

    Returns:
        (Plan): The prior plan, if no changes were detected, or
            a newly stored plan object containing:
                - id: Newly assigned identifier.
                - revision_of: Identifier of the prior plan.
                - task_prompt: Prompt describing the revision.
                - primitive_plan: Revised list of primitives.
    """

    revision_of = req.revision_of
    task_prompt = req.task_prompt
    primitive_plan = req.primitive_plan

    primitive_plan = [step.model_dump() for step in primitive_plan]


    prior_version = get_json(revision_of, JSON_DIR)

    # Don't save if same as revision
    # TODO: Make sure this is working properly
    if prior_version['primitive_plan'] == primitive_plan:
        return prior_version 
    

    data_to_store = {
        'id': None,  # Added in store_json
        'revision_of': revision_of,
        'task_prompt': task_prompt,
        'primitive_plan': primitive_plan
    }

    stored_data = store_json(data_to_store, JSON_DIR)
    return stored_data



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



    primitive_plan = [step.model_dump() for step in primitives]

    app.state.bridge_node.trigger_primitives(primitive_plan, start_index, on_real=on_real)


    return {'success': True, 'executed_on': 'real' if on_real else 'sim'}



@app.get("/api/primitive_plan/id/{item_id}", response_model=Plan)
def get_plan(item_id: str) -> Plan:
    """
    Retrieve a previously stored primitive plan by identifier.

    Args:
        item_id (str): Identifier of the stored primitive plan
            (filename without extension).

    Returns:
        (Plan): The plan object containing:
                - id: Newly assigned identifier.
                - revision_of: Identifier of the prior plan.
                - task_prompt: Prompt describing the revision.
                - primitive_plan: Revised list of primitives.
    """
    return get_json(item_id, JSON_DIR)


@app.get("/api/primitive_plan/latest", response_model=Optional[Plan])
def get_latest_plan() -> Optional[Plan]:
    """
    Retrieve the most recently stored primitive plan.
    Returns:
        (Plan): The plan object containing:
                - id: Newly assigned identifier.
                - revision_of: Identifier of the prior plan.
                - task_prompt: Prompt describing the revision.
                - primitive_plan: Revised list of primitives.
    """
    return get_latest_json(JSON_DIR)


@app.get("/api/primitive_plan/all", response_model=List[Plan])
def get_all_plans() -> List[Plan]:
    """
    Retrieves all the primitive plans
    Returns:
        (List[Plan]): List of plan objects containing:
                - id: Newly assigned identifier.
                - revision_of: Identifier of the prior plan.
                - task_prompt: Prompt describing the revision.
                - primitive_plan: Revised list of primitives.
    """


    return get_all_json(JSON_DIR)


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

    return app.state.bridge_node.get_cur_executing_idx()



@app.post("/api/primitive_plan/cancel")
def stop_plan_execution():
    """
    Cancels the currently executing primitive plan.
    """
    app.state.bridge_node.cancel_primitives_goal()
    return {'status': 'cancel message sent'}


@app.post("/api/primitive_scene/reset")
def reset_primitive_scene(prim_idx: List[int] = Query(None, 
        description="Hierarchical index to reset to as array, e.g. [0,1,2]")):
    
    """
    Restores the scene to the recorded state at the start of the given primitive in the 
    flattened plan was executed. Restores both objects and joint positions.

    The scene state must have been previously recorded for the given
    primitive index. If no state exists, the function exits without
    making any changes.

    Args:
        prim_idx (list): HIERARCHICAL index of the primitive in the
            plan whose post-execution scene state should be restored.
    """

    app.state.bridge_node.reset_primitive_scene(prim_idx)
    
    return {'status': 'reset request sent'}


