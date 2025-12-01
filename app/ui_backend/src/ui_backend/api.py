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

class PrimitiveActionClientNode(Node):
    def __init__(self):
        """
        TODO
        """
        super().__init__('backend_primitive_client')
        self.client = ActionClient(self, Primitives, '/primitives')

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
        goal_msg = Primitives.Goal()
        goal_msg.primitives = []
        for prim in primitives:
            prim_msg = Primitive()
            prim_msg.type = prim['type']

            if 'arm' in prim:
                prim_msg.arm = prim['arm']
            
            if 'pose' in prim:
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


