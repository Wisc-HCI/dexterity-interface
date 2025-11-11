
from robot_motion_interface.isaacsim.isaacsim_interface import IsaacsimInterface, IsaacsimControlMode
import argparse  # IsaacLab requires using argparse
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path
import numpy as np


class ObjectType(Enum):
    """
    Supported Object types.
    """
    CUBE = 'cube'


@dataclass
class Object:
    """
    Object instance in the IsaacSim scene.

    Attributes:
        type (ObjectType): The type of object to create. 
        scene_path (str): Unique USD prim path for this object in the scene graph. 
        position (list[float]): The world position [x, y, z] in meters.
        rotation (list[float]): Quaternion orientation [qx, qy, qz, qw]
        size (list[float]): The dimensions [x, y, z] of the object in meters. 
        mass (float): Physical mass of the object in kilograms. 
        collision (bool): True if collision enabled on object
    """
    type: ObjectType = ObjectType.CUBE
    scene_path: str = field(default_factory=lambda: f"/World/{ObjectType.CUBE.value}")
    position: list = field(default_factory=lambda: [0.0, 0.0, 0.0])
    rotation: list = field(default_factory=lambda: [0.0, 0.0, 0.0, 1.0])
    size: list = field(default_factory=lambda: [0.1, 0.1, 0.1])
    mass: float = 0.1
    collision: bool = True





class IsaacsimObjectInterface(IsaacsimInterface):
    def __init__(self, urdf_path:str, ik_settings_path:str, joint_names: list[str], home_joint_positions:np.ndarray,
                base_frame:str, ee_frames:list[str],
                kp: np.ndarray, kd:np.ndarray, control_mode: IsaacsimControlMode,
                num_envs:int = 1, device: str = 'cuda:0', headless:bool = False, parser: argparse.ArgumentParser = None):
        """
        Isaacsim Interface  extension for running the simulation with accessors for using IsaacSim with object
        interactions (moving objects, resetting the environment, etc.).

        Args:
            urdf_path (str): Path to urdf, relative to robot_motion_interface/ (top level).
            ik_settings_path (str): Path to ik settings yaml 
            joint_names (list[str]): (n_joints) Ordered list of joint names for the robot.
            home_joint_positions (np.ndarray): (n_joints) Default joint positions (rads)
            base_frame (str): Base frame name for which cartesian poses of end-effector(s) are relative to
            ee_frames (list[str]): (e,) List of frame names for each end-effector
            kp (np.ndarray): (n_joints) Joint proportional gains (array of floats).
            kd (np.ndarray): (n_joints) Joint derivative gains (array of floats).
            control_mode (IsaacsimControlMode): Control mode for the robot (e.g., JOINT_TORQUE).
            num_envs (int): Number of environments to spawn in simulation. Default is 1.
            device (str): Device identifier (e.g., "cuda:0" or "cpu"). Default is "cuda:0".
            headless (bool): If True, run without rendering a viewer window. Default is False.
            parser (ArgumentParser): 
                An existing argument parser to extend. NOTE: If you use parser in a script that calls this one,
                you WILL need to pass the parser, or this will error. If None, a new parser will be created.
        """
        
        super().__init__(urdf_path, ik_settings_path, joint_names, home_joint_positions,
            base_frame, ee_frames, kp, kd, control_mode, num_envs, device, headless, parser)

        self._objects_to_add = []
        self._initialized_objects = []
        self._object_idx = 0

    def freeze(self):
        """
        TODO
        """
        ...


    def unfreeze(self):
        """
        TODO
        """
        ...


    def place_objects(self, objects: list[Object]):
        """
        Initialize list of objects in Isaacsim
        Args:
            objects (list[Object]): List of objects 
        """
        self._objects_to_add.extend(objects)

    def move_object(self):
        """
        TODO
        """
        ...
    
    def _load_objects(self):
        """
        Loads objects into isaacsim
        Args:
            objects (list[Object]): List of objects 
        """

        if not self._objects_to_add:
            return

        # Must be imported after loop is launched
        from isaaclab.assets import RigidObject
        from isaaclab.assets import RigidObjectCfg
        import isaaclab.sim as sim_utils


        for obj in self._objects_to_add:
            
            # Make sure its unique
            obj.scene_path += f"_{self._object_idx}"

            if obj.type == ObjectType.CUBE:

                cube = RigidObjectCfg(
                    prim_path=obj.scene_path,
                    spawn=sim_utils.CuboidCfg(
                        size=tuple(obj.size),
                        mass_props=sim_utils.MassPropertiesCfg(mass=obj.mass), 
                        rigid_props=sim_utils.RigidBodyPropertiesCfg(
                            rigid_body_enabled=True,
                            kinematic_enabled=False,
                        ),
                        collision_props=sim_utils.CollisionPropertiesCfg(collision_enabled=obj.collision),
                    ),
                    init_state=RigidObjectCfg.InitialStateCfg(pos=tuple(obj.position), rot=tuple(obj.rotation))
                )
                RigidObject(cfg=cube)

            self._initialized_objects.append(obj)
            self._object_idx += 1

        self._objects_to_add = [] # Clear objects since added

        

    def _post_step(self, env: "ManagerBasedEnv", obs: dict):
        """
        (Hook) Called after simulation _step to load objects
        Args:
            env (ManagerBasedEnv): The active simulation environment.
            obs (dict): The raw observation dictionary from the environment.
        """

        # Don't overwrite parent
        super()._post_step(env, obs)

        # Load newly added objects
        self._load_objects()
        



if __name__ == "__main__":

    config_path = Path(__file__).resolve().parents[3] / "config" / "isaacsim_config.yaml"

    isaac = IsaacsimObjectInterface.from_yaml(config_path)
    isaac.start_loop()