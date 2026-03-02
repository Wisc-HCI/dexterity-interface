from ui_backend.schemas import Primitive, Execution, Plan, NewPlan, RevisedPlan, Pose, LogEvent
from ui_backend.utils.UIBridgeNode import UIBridgeNode, RosRunner
from ui_backend.utils.utils import store_json, get_latest_json, get_json, get_all_json, json_equal, append_log, ct_timestamp
from primitives_ros.utils.create_high_level_prims import parse_prim_plan
from planning.llm.gpt import GPT
from planning.llm.primitive_breakdown import PrimitiveBreakdown

from pathlib import Path
from typing import List, Optional
from contextlib import asynccontextmanager
import time
from fastapi import FastAPI, Query
from fastapi.middleware.cors import CORSMiddleware
from typing import List
import os

import rclpy

#######################################################
############## Envs (Configurable) ####################
TEST       = int(os.environ.get("TEST", 0))                                                                                                                            
USE_VISION = os.environ.get("USE_VISION", "true").lower() == "true"    

# Experiment
TASK    = int(os.environ.get("TASK", 0))
PID     = int(os.environ.get("PID", 0))


#########################################################
####################### CONSTANTS #######################
if PID == 0:
    PARENT = Path(__file__).resolve().parent
    PLAN_DIRECTORY = PARENT / "json_primitives"
    LOG_DIRECTORY = PARENT / "logs"
else:
    PARENT = Path(__file__).resolve().parent.parent.parent.parent
    DIRECTORY =  PARENT / f"experiment_logging" / f"PID_{PID}" / f"task_{TASK}_t{ct_timestamp()}"
    PLAN_DIRECTORY =  DIRECTORY / "plans"
    LOG_DIRECTORY = DIRECTORY 


PLAN_DIRECTORY.mkdir(parents=True, exist_ok=True)
LOG_DIRECTORY.mkdir(parents=True, exist_ok=True)

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
    PLAN_DIRECTORY.mkdir(exist_ok=True)

    # "Global" variables

    if not rclpy.ok():
        rclpy.init()

    app.state.runner = RosRunner()
    app.state.bridge_node = UIBridgeNode(USE_VISION, TASK)
    app.state.ui_marker_spawned = False
    
    app.state.gpt = GPT(
        "You are a precise robot task planner. Always return valid JSON.\n\n"
        "COORDINATE SYSTEM:\n"
        "- Right is +x, Forward is +y, Up is +z\n"
        "- ALL poses should have orientation: [qx, qy, qz, qw] = [1, 0, 0, 0]\n\n"
        "ROBOT SETUP:\n"
        "- Left arm mounted at [-0.5, -0.09, 0.9] (x, y, z in m)\n"
        "- Right arm mounted at [0.5, -0.09, 0.9] (x, y, z in m)\n"
        "- Use the left arm for objects at x < 0, right arm for objects at x >= 0\n"
        "- Table center: [0.0, 0.0, 0.9144], dimensions: [1.8288, 0.62865, 0.045] (x, y, z in m)\n\n"
        "PLANNING RULES:\n"
        "- Always start every plan with a home primitive\n"
        "- Prefer mid and high level primitives over low level ones\n"
        "- grasp_pose x,y,z must match the object's pose x,y,z from objects_in_scene\n"
        "- Use left arm (x<0 target) and right arm (x>=0 target) as needed in the same plan\n\n"
        "EXAMPLE - 'set the table' (bowl at [0.2,-0.2,0.95], cup at [0.2,-0.05,0.95], "
        "spoon at [-0.1,0.1,0.95], fork at [-0.2,0.1,0.95]):\n"
        "{\"primitive_plan\": ["
        "{\"name\": \"home\", \"parameters\": {}},"
        "{\"name\": \"pick_and_place\", \"parameters\": {\"arm\": \"right\", \"grasp_pose\": [0.2, -0.2, 0.95, 1, 0, 0, 0], \"end_position\": [0.0, -0.15, 0.95], \"object\": \"bowl\"}},"
        "{\"name\": \"pick_and_place\", \"parameters\": {\"arm\": \"right\", \"grasp_pose\": [0.2, -0.05, 0.95, 1, 0, 0, 0], \"end_position\": [0.1, -0.05, 0.95], \"object\": \"cup\"}},"
        "{\"name\": \"pick_and_place\", \"parameters\": {\"arm\": \"left\", \"grasp_pose\": [-0.1, 0.1, 0.95, 1, 0, 0, 0], \"end_position\": [-0.1, -0.15, 0.95], \"object\": \"spoon\"}},"
        "{\"name\": \"pick_and_place\", \"parameters\": {\"arm\": \"left\", \"grasp_pose\": [-0.2, 0.1, 0.95, 1, 0, 0, 0], \"end_position\": [-0.2, -0.15, 0.95], \"object\": \"fork\"}}"
        "]}"

        ,save_history=False)

    app.state.planner = PrimitiveBreakdown(app.state.gpt, PRIMS_PATH)

    # Start ROS Node
    app.state.runner.start(app.state.bridge_node)

    # Make sure sim is cleared
    time.sleep(0.5)
    app.state.bridge_node.remove_all_objects()
    app.state.bridge_node.home_sim_joint_positions()  # Reset robot so scene can be viewed

    # When app closes
    yield
    if app.state.runner is not None:
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
def spawn_objects(force: bool = Query(True, description="Force spawn all objects regardless of position change")):
    """
    Call to initialize objects in the scene
    """
    app.state.bridge_node.spawn_objects(force=force)
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
        prior_version = get_json(revision_of, PLAN_DIRECTORY)

    scene = app.state.bridge_node.get_scene(False)
    if TEST == 0:
        plan = app.state.planner.plan(task_prompt, scene, prior_version)
        print("-----------plan----")
        print(plan)
    else:
        plan = test_llm_plan(TEST)

    high_level_plan = plan.get("primitive_plan", [])  # TODO:Revert after debugging
    

    parsed_out_plan = parse_prim_plan(high_level_plan, scene, repair_parameters=True, repair_collision=True)

    data_to_store = {
        'id': None,
        'revision_of': revision_of,
        'task_prompt': task_prompt,
        'primitive_plan': parsed_out_plan
    }
    return store_json(data_to_store, PLAN_DIRECTORY)
        
def test_llm_plan(test):
    """
    TODO: Move to other file??
    """
    if test == 1:
        # Move all objects to center with pick and place
        return {'primitive_plan': [{'name': 'home', 'parameters': {}},
            {'name': 'release', 'parameters': {'arm': 'right'}},
            {'name': 'release', 'parameters': {'arm': 'left'}},
            {'name': 'pick_and_place', 'parameters': {'arm': 'right', 'grasp_pose': [0.2, -0.2, 0.98, 0.707, 0.707, 0.0, 0.0], 'end_position': [0.1, -0.2, 0.95], 'object': 'bowl_1'}},
            {'name': 'pick_and_place', 'parameters': {'arm': 'right', 'grasp_pose': [0.2, 0.11, 0.98, 0.707, 0.707, 0.0, 0.0], 'end_position': [0.0, 0.07, 0.95], 'object': 'cup_1'}},
            {'name': 'pick_and_place', 'parameters': {'arm': 'right', 'grasp_pose': [0.1, 0.01, 0.98, 0.707, 0.707, 0.0, 0.0], 'end_position': [0.0, -0.07, 0.95], 'object': 'cup_2'}},
            {'name': 'pick_and_place', 'parameters': {'arm': 'left', 'grasp_pose': [-0.3, -0.2, 0.98, 0.707, 0.707, 0.0, 0.0], 'end_position': [-0.1, 0.0, 0.95], 'object': 'bowl_1'}},
            {'name': 'pick_and_place', 'parameters': {'arm': 'left', 'grasp_pose': [-0.3, 0.11, 0.98, 0.707, 0.707, 0.0, 0.0], 'end_position': [0.0, 0.0, 0.95], 'object': 'cup_1'}}
            ]}

    elif test == 2:

        return {'primitive_plan': [{'name': 'home', 'parameters': {}},
            {'name': 'release', 'parameters': {'arm': 'right'}},
            {'name': 'release', 'parameters': {'arm': 'left'}},
            {'name': 'pick_and_place', 'parameters': {'arm': 'right', 'grasp_pose': [0.2, -0.2, 0.98, 0.707, 0.707, 0.0, 0.0], 'end_position': [0.1, -0.2, 0.95], 'object': 'bowl_1'}},
            {'name': 'pick_and_place', 'parameters': {'arm': 'right', 'grasp_pose': [0.2, 0.11, 0.98, 0.707, 0.707, 0.0, 0.0], 'end_position': [0.0, 0.07, 0.95], 'object': 'cup_1'}},
            ]}
    # Pour cup into bowl
    elif test == 3:
        return {'primitive_plan': [ {'name': 'home', 'parameters': {}}, 
            {'name': 'pick', 'parameters': {'arm': 'right', 'grasp_pose': [0.2, 0.1, 0.95, 0.0, 0.0, 0.0, 1.0], 'end_position': [0.0, 0.0, 0.95], 'object': 'cup_1'}}, 
            {'name': 'pour', 'parameters': {'arm': 'right', 'initial_pose': [0.0, 0.0, 0.95, 0.0, 0.0, 0.0, 1.0], 'pour_orientation': [0.0, 0.0, 0.0, 1.0], 'pour_hold': 2.0, 'object': 'cup', 'receiving_object': 'bowl_1'}}]}

    # Set the table
    elif test == 4:
        return {'primitive_plan': [
            {'name': 'home', 'parameters': {}}, 
            {'name': 'pick_and_place', 'parameters': {'arm': 'right', 'grasp_pose': [0.2, -0.05, 0.95, 1, 0, 0, 0], 'end_position': [0.0, -0.05, 0.9369], 'object': 'cup_1'}}, 
            {'name': 'pick_and_place', 'parameters': {'arm': 'right', 'grasp_pose': [0.2, -0.2, 0.95, 1, 0, 0, 0], 'end_position': [0.0, -0.2, 0.9369], 'object': 'bowl_1'}}, 
            {'name': 'pick_and_place', 'parameters': {'arm': 'right', 'grasp_pose': [0.2, 0.1, 0.95, 1, 0, 0, 0], 'end_position': [-0.1, -0.2, 0.9369], 'object': 'spoon_1'}}, 
            {'name': 'pick_and_place', 'parameters': {'arm': 'right', 'grasp_pose': [0.1, 0.1, 0.95, 1, 0, 0, 0], 'end_position': [0.2, -0.2, 0.9369], 'object': 'fork_1'}}]}
    
    elif test==5:
        return {'primitive_plan': [
            {'name': 'home', 'parameters': {}}, 
            {'name': 'pick_and_place', 'parameters': {'arm': 'left', 'grasp_pose': [-0.1011597141623497, -0.1203981414437294, 0.945, 0.0, 0.0, 0.0, 1.0], 'end_position': [-0.2011597141623497, -0.1203981414437294, 0.945], 'object': 'cup_1'}}, 
            # {'name': 'pick_and_place', 'parameters': {'arm': 'right', 'grasp_pose': [0.03994744271039963, -0.03241425007581711, 0.945, 0.0, 0.0, 0.0, 1.0], 'end_position': [-0.06005255728960037, -0.03241425007581711, 0.945], 'object': 'bowl_1'}}, 
            # {'name': 'pick_and_place', 'parameters': {'arm': 'right', 'grasp_pose': [0.03281160071492195, 0.12902496755123138, 0.945, 0.0, 0.0, 0.707, 0.707], 'end_position': [-0.06718839928507805, 0.12902496755123138, 0.945], 'object': 'spoon_1'}}, 
            # {'name': 'pick_and_place', 'parameters': {'arm': 'right', 'grasp_pose': [0, 0, 0, 0, 0, 0, 1], 'end_position': [-0.1, 0, 0], 'object': 'fork'}}
            ]}

    else:
        return {'primitive_plan': []}

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


    prior_version = get_json(revision_of, PLAN_DIRECTORY)

    # Don't save if same as revision
    if json_equal(prior_version['primitive_plan'], primitive_plan):
        return prior_version 
    

    data_to_store = {
        'id': None,  # Added in store_json
        'revision_of': revision_of,
        'task_prompt': task_prompt,
        'primitive_plan': primitive_plan
    }

    stored_data = store_json(data_to_store, PLAN_DIRECTORY)
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
    return get_json(item_id, PLAN_DIRECTORY)


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
    return get_latest_json(PLAN_DIRECTORY)


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


    return get_all_json(PLAN_DIRECTORY)


@app.post("/api/primitive_plan/reparse", response_model=List[Primitive])
def reparse_primitive_plan(
    primitive_plan: List[Primitive],
    repair_parameters: bool = Query(True),
    repair_collision: bool = Query(True),
) -> List[Primitive]:
    """
    Given a full primitive plan, re-parses it into core primitives.
    Args:
        primitive_plan (List[Primitive]): The full plan to re-parse.
        repair_parameters (bool): If True, auto-fill/correct primitive parameters.
        repair_collision (bool): If True, auto-insert retracts on arm collision.

    Returns:
        (List[Primitive]): The regenerated plan.
    """

    scene = app.state.bridge_node.get_scene()
    regenerated_plan = parse_prim_plan(
        [p.model_dump() for p in primitive_plan],
        scene,
        repair_parameters=repair_parameters,
        repair_collision=repair_collision,
    )

    return regenerated_plan


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


@app.post("/api/scene/freeze")
async def freeze_scene():
    """
    Locks the most recently captured YOLO scene in the backend.
    While frozen, both the planner and simulator use this cached scene
    instead of re-running YOLO.
    """
    app.state.bridge_node.freeze_scene()
    return {'frozen': True}


@app.post("/api/scene/unfreeze")
async def unfreeze_scene():
    """
    Clears the frozen scene so the planner and simulator run YOLO again.
    """
    app.state.bridge_node.unfreeze_scene()
    return {'frozen': False}


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


@app.post("/api/ui_marker/spawn")
def ui_marker_spawn(req: Pose):
    """
    Spawns/activates a UI marker object in simulation at the given pose.
    """

    app.state.bridge_node.spawn_object("marker", req.pose)
    
    return {"success": True}


@app.post("/api/ui_marker/move")
def ui_marker_move(req: Pose):
    """
    Moves the UI marker object in simulation to the given pose.
    """

    app.state.bridge_node.move_object("marker", req.pose)
    return {"success": True}


@app.post("/api/ui_marker/remove")
def ui_marker_remove():
    """
    Removes the UI marker from the simulation by hiding it and moving it to the origin.
    """
    app.state.bridge_node.remove_object("marker")
    app.state.ui_marker_spawned = False
    return {"success": True}


@app.post("/api/log")
def log_event(req: LogEvent):
    """
    Appends a frontend event to the session log file.

    Args:
        req (LogEvent): Payload containing:
            - event (str): Event name (e.g. 'plan_submitted').
            - data (dict): Arbitrary event payload.
    """
    append_log(req.event, req.data, LOG_DIRECTORY / "events.jsonl")
    return {"success": True}
