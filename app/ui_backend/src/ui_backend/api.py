import threading
from pathlib import Path
from typing import Optional, List

from planning.llm.gpt import GPT
from planning.llm.primitive_breakdown import PrimitiveBreakdown

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel

# ROS
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from primitive_msgs_ros.action import Primitives as PrimitivesAction
from primitive_msgs_ros.msg import Primitive as PrimitiveMsg 
from geometry_msgs.msg import PoseStamped       



# TEST
rclpy.init()

class PrimitiveActionClientNode(Node):
    def __init__(self):
        """
        TODO
        """
        super().__init__('backend_primitive_client')
        self.client = ActionClient(self, PrimitivesAction, '/primitives')

    def trigger_primitives(self, primitives:list[dict]):
        """
        Sends primitive actions to execute on robot.
        Args:
            primitives (list[dict]): List of primitives dicts which each dict
                can be of format of:
                {'type': 'prim_type',
                 'arm': 'left_or_right',
                 'pose': np.array([x, y, z, qx, qy, qz, qw])
                }
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
        
        self.client.wait_for_server()
        future = self.client.send_goal_async(goal_msg)
        return future

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



# TODO: make this modular
app = FastAPI()


def get_current_scene():
    # TODO
    return [
        {"name": "block", "description": "blue block", "position": [0.0, 0.0, 0.0]},

    ]


# Allow your frontend to call the API during dev
app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://127.0.0.1:5500",],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

class Task(BaseModel):
    task: str

class Primitive(BaseModel):
    type: str
    arm: Optional[str] = None
    pose: Optional[list[float]] = None
    
@app.get("/api/test", )
def get_test():
    """
    TODO
    """
    prims = [
        {'type': 'home'},
        {'type': 'move_to_pose', 'arm': 'left', 'pose': [-0.2, 0.2, 0.2, 0.707, 0.707, 0.0, 0.0 ]},
        {'type': 'envelop_grasp', 'arm': 'left'},
        {'type': 'release', 'arm': 'left'},
        {'type': 'envelop_grasp', 'arm': 'right'},
        {'type': 'release', 'arm': 'right'}
    ]
    ros_node.trigger_primitives(prims)
    
    return {"data": "test successful"}


@app.post("/api/primitive_plan", response_model=List[Primitive])
def primitive_plan(data: Task):
    """TODO"""

    task = data.task
    scene = get_current_scene()
    plan = planner.plan(task, scene)
   

    #TODO: CLEAN
    remapped = []
    for step in plan['primitive_plan'][0]['low_level_primitive_ordering']:
        new_step = {
            "type": step["primitive_name"],
            **step.get("parameters", {})
        }
        remapped.append(new_step)

    print("PLAN", remapped)

    return remapped


@app.post("/api/execute_plan")
def execute_plan(data: List[Primitive]):
    """TODO"""
    primitives = [step.model_dump() for step in data]
    ros_node.trigger_primitives(primitives)
    return {'success': True}

