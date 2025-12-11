import threading
from pathlib import Path
from typing import Optional, List

from planning.llm.gpt import GPT
from planning.llm.primitive_breakdown import PrimitiveBreakdown

from fastapi import FastAPI, Query
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel

# ROS
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from primitive_msgs_ros.action import Primitives as PrimitivesAction
from primitive_msgs_ros.msg import Primitive as PrimitiveMsg 
from geometry_msgs.msg import PoseStamped       
import ulid
import json


# TEST
rclpy.init()

# TODO: NAME THIS BETTER
class PrimitiveActionClientNode(Node):
    def __init__(self):
        """
        TODO
        """
        super().__init__('backend_primitive_client')
        self.sim_client = ActionClient(self, PrimitivesAction, '/primitives')
        self.real_client = ActionClient(self, PrimitivesAction, '/primitives/real')

        self.spawn_obj_pub = self.create_publisher(PoseStamped, "/spawn_object", 10)
        self.move_obj_pub = self.create_publisher(PoseStamped, "/move_object", 10)

    def trigger_primitives(self, primitives:list[dict], on_real:bool=False):
        """
        Sends primitive actions to execute on robot.
        Args:
            primitives (list[dict]): List of primitives dicts which each dict
                can be of format of:
                {'type': 'prim_type',
                 'arm': 'left_or_right',
                 'pose': np.array([x, y, z, qx, qy, qz, qw])
                }
            on_real (bool): True if execute on real, else False.
        """
        
        goal_msg = PrimitivesAction.Goal()
        goal_msg.primitives = []
        for prim in primitives:
            prim_msg = PrimitiveMsg()
            prim_msg.type = prim['type']

            if prim.get("arm") is not None:
                prim_msg.arm = prim['arm']
            
            if prim.get("pose") is not None:
                x, y, z, qx, qy, qz, qw = prim['pose']
                pose_msg = PoseStamped()
                pose_msg.pose.position.x = float(x)
                pose_msg.pose.position.y = float(y)
                pose_msg.pose.position.z = float(z)
                pose_msg.pose.orientation.x = float(qx)
                pose_msg.pose.orientation.y = float(qy)
                pose_msg.pose.orientation.z = float(qz)
                pose_msg.pose.orientation.w = float(qw)

                prim_msg.pose = pose_msg

            goal_msg.primitives.append(prim_msg)
        
        client = self.real_client if on_real else self.sim_client
        client.wait_for_server()
        future = client.send_goal_async(goal_msg)
        return future
   
    ######################## OBJECTS ########################

    def spawn_object(self, object_handle: str, pose:list):
        """
        TODO
        Equivalent to:
        ros2 topic pub /spawn_object geometry_msgs/PoseStamped --once
        """

        msg = self._make_pose_stamped(object_handle, pose)
        self.spawn_obj_pub.publish(msg)


    def move_object(self, object_handle: str, pose:list):
        """
        TODO
        Equivalent to:
        ros2 topic pub /spawn_object geometry_msgs/PoseStamped --once
        """

        msg = self._make_pose_stamped(object_handle, pose)
        self.move_obj_pub.publish(msg)

      
    def _make_pose_stamped(self, object_handle: str, pose: tuple | list) -> PoseStamped:
        """
        TODO: Clean up
        pose = (x, y, z, qx, qy, qz, qw)
        """
        msg = PoseStamped()
        msg.header.frame_id = object_handle

        msg.pose.position.x = float(pose[0])
        msg.pose.position.y = float(pose[1])
        msg.pose.position.z = float(pose[2])

        msg.pose.orientation.x = float(pose[3])
        msg.pose.orientation.y = float(pose[4])
        msg.pose.orientation.z = float(pose[5])
        msg.pose.orientation.w = float(pose[6])

        return msg
    
ros_node = PrimitiveActionClientNode()

def spin_ros():
    """
    TODO
    """
    rclpy.spin(ros_node)

threading.Thread(target=spin_ros, daemon=True).start()

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

    return [
        {"name": "cup", "description": "small cup", "position": [0.3, 0.2, 0.95, 0.0, 0.0, 0.0, 1.0]},
        {"name": "bowl", "description": "bowl", "position": [ 0.3, -0.2, 0.95, 0.0, 0.0, 0.0, 1.0]},
    ]



# TODO: make this modular
app = FastAPI()

# Allow your frontend to call the API during dev
app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:3000",],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


SCENE = get_current_scene()
import time
time.sleep(1)
for obj in SCENE:
    ros_node.spawn_object(obj["name"], obj["position"])



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
#     ros_node.trigger_primitives(prims)
    
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
        ros_node.move_object(obj["name"], obj["position"])


    primitive_plan = [step.model_dump() for step in primitives]
    ros_node.trigger_primitives(primitive_plan,  on_real=on_real)
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
