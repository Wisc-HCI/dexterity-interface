import threading

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel

# ROS
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from primitive_msgs_ros.action import Primitives
from primitive_msgs_ros.msg import Primitive      
from geometry_msgs.msg import PoseStamped       



# TEST
rclpy.init()

class PrimitiveActionClient(Node):
    def __init__(self):
        """
        TODO
        """
        super().__init__('fastapi_primitive_client')
        self.client = ActionClient(self, Primitives, '/primitives')

    def send(self, primitive_type: str, arm: str):
        """
        TODO
        """
        goal_msg = Primitives.Goal()
        
        prim = Primitive()
        prim.type = primitive_type
        prim.arm = arm


        goal_msg.primitives = [prim] 
        self.client.wait_for_server()
        future = self.client.send_goal_async(goal_msg)
        return future

ros_node = PrimitiveActionClient()

def spin_ros():
    """
    TODO
    """
    rclpy.spin(ros_node)

threading.Thread(target=spin_ros, daemon=True).start()




# TODO: make this modular
app = FastAPI()

# Allow your frontend to call the API during dev
app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://127.0.0.1:5500",],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

class EchoIn(BaseModel):
    text: str

class EchoOut(BaseModel):
    echoed: str

@app.get("/api/test")
def get_test():
    """
    TODO
    """
    ros_node.send('envelop_grasp', 'left')
    
    return {"data": "test successful"}


