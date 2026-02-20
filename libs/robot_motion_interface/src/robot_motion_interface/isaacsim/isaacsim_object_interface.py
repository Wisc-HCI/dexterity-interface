
from robot_motion_interface.isaacsim.isaacsim_interface import IsaacsimInterface, IsaacsimControlMode
import argparse  # IsaacLab requires using argparse
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path
import numpy as np
import torch


USD_DIR = Path(__file__).resolve().parent / "usds"


# TODO: HANDLE geometry and usd shapes differently

class ObjectHandle(Enum):
    """
    Supported Object handles.
    """
    CUBE = 'cube'
    CYLINDER = 'cylinder'
    SPHERE = 'sphere'
    BARRIER = 'barrier'
    SPOON = 'spoon'
    FORK = 'fork'

    # usd
    BOWL = 'bowl'
    CUP = 'cup'

    
    


@dataclass
class Object:
    """
    Object instance in the IsaacSim scene.

    Attributes:
        handle (str): Name/Handle of the object to create. Must be unique and in the form of `bowl`
            or `bowl_1`where the str before the underscore is an ObjectHandle
        position (list[float]): The world position [x, y, z, qx, qy, qz, qw]. Position in meters.
    """
    handle: str = 'cube'
    type: ObjectHandle = None
    pose: list = field(default_factory=lambda: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]) 

    def __post_init__(self):
        """
        Determines type by parsing handle
        """
        if self.type:
            return
        
        parts = self.handle.split("_", 1)

        # Parse type
        try:
            self.type = ObjectHandle(parts[0])
        except ValueError as exc:
            raise ValueError(
                f"Invalid object handle '{self.handle}'. "
                f"Expected one of {[h.value for h in ObjectHandle]}"
            )


class IsaacsimObjectInterface(IsaacsimInterface):
    def __init__(self, urdf_path:str, ik_settings_path:str, joint_names: list[str], home_joint_positions:np.ndarray,
                base_frame:str, ee_frames:list[str], target_tolerance:float,
                kp: np.ndarray, kd:np.ndarray, max_joint_delta:float, control_mode: IsaacsimControlMode,
                num_envs:int = 1, device: str = 'cuda:0', headless:bool = False, parser: argparse.ArgumentParser = None):
        """
        Isaacsim Interface for running the simulation with accessors for setting
        setpoints of custom controllers.

        Args:
            urdf_path (str): Path to urdf, relative to robot_motion_interface/ (top level).
            ik_settings_path (str): Path to ik settings yaml 
            joint_names (list[str]): (n_joints) Ordered list of joint names for the robot.
            home_joint_positions (np.ndarray): (n_joints) Default joint positions (rads)
            base_frame (str): Base frame name for which cartesian poses of end-effector(s) are relative to
            ee_frames (list[str]): (e,) List of frame names for each end-effector
            target_tolerance(float): Threshold (rads) that determines how close the robot's joints must be 
                to the commanded target to count as reached.
            kp (np.ndarray): (n_joints) Joint proportional gains (array of floats).
            kd (np.ndarray): (n_joints) Joint derivative gains (array of floats).
            max_joint_delta (float): Caps the joint delta per control step
                to smooth motion toward the setpoint (in radians). If negative (e.g., -1), the limit is ignored.
            control_mode (IsaacsimControlMode): Control mode for the robot (e.g., JOINT_TORQUE).
            num_envs (int): Number of environments to spawn in simulation. Default is 1.
            device (str): Device identifier (e.g., "cuda:0" or "cpu"). Default is "cuda:0".
            headless (bool): If True, run without rendering a viewer window. Default is False.
            parser (ArgumentParser): 
                An existing argument parser to extend. NOTE: If you use parser in a script that calls this one,
                you WILL need to pass the parser, or this will error. If None, a new parser will be created.
        """
        super().__init__(urdf_path, ik_settings_path, joint_names, home_joint_positions, 
                base_frame, ee_frames, target_tolerance, 
                kp, kd, max_joint_delta, control_mode, 
                num_envs, device, headless, parser)

        self._objects_to_add = []
        self._objects_to_move = {}
        self._initialized_objects = []
        self._object_poses = {}

        



    # TODO: COMBINE MOVE AND PLACE
    def place_objects(self, objects: list[Object]):
        """
        Initialize list of objects in Isaacsim
        Args:
            objects (list[Object]): List of objects 
        """
        self._objects_to_add.extend(objects)

    

    def move_object(self, object_handle:str, pose:np.ndarray):
        """
        Update the pose of an existing object in the Isaac Sim scene.

        Args:
            object_handle (str): Unique identifier of the object
                to be moved.
            pose (np.ndarray): (7,) Target pose of the object [x,y,z,qx,qy,qz,qw]
        """
        self._objects_to_move[object_handle] = pose


    def get_object_poses(self) -> dict[str, np.ndarray]:
        """
        Get world poses of all initialized objects.

        Returns:
            (dict[str, np.ndarray]): Mapping from object handle -> pose [x, y, z, qx, qy, qz, qw] (m,rad)
        """
        return self._object_poses    
    

    def get_object_pose(self, handle:str) -> np.ndarray:
        """
        Get world poses of given object.
        Args:
            handle (str): Unique ID of object
        Returns:
            (np.ndarray): pose [x, y, z, qx, qy, qz, qw] in (m, rad)
        """  
        return self._object_poses[handle]
    
    
    # def _load_objects(self):
    #     """
    #     Loads objects into isaacsim
    #     Args:
    #         objects (list[Object]): List of objects 
    #     """

    #     if not self._objects_to_add:
    #         return

    #     # TODO: Add check for duplicates

    #     for obj in self._objects_to_add:
    #         handle_str = obj.handle.value
    #         obj_sim = self.env.scene[handle_str]
    #         self.move_object(handle_str, obj.pose)
    #         obj_sim.set_visibility(True, [0]) # Breaks if leave the env blank

    #         self._initialized_objects.append(obj)            

    #     self._objects_to_add = [] # Clear objects since added


    def _get_scene_object(self, handle: str):
        """
        Resolve an object handle to either:
        1) a scene-managed object, or
        2) a dynamically spawned object
        """

        try:
            return self.env.scene[handle]
        except KeyError:
            pass

        raise KeyError(
            f"Object '{handle}' not found in scene or dynamic registry."
        )
        
    def _load_objects(self):
        """
        Loads objects into isaacsim
        Args:
            objects (list[Object]): List of objects 
        """

        if not self._objects_to_add:
            return


        for obj in self._objects_to_add:
            
            env_obj = self.env.scene[obj.handle]
            self.move_object(obj.handle, obj.pose,)
            env_obj.set_visibility(True, [0]) # Breaks if leave the env blank

            self._initialized_objects.append(obj)


        self._objects_to_add = [] # Clear objects since added

    def _move_objects(self):
        if not self._objects_to_move:
            return
        
        obj_list = list(self._objects_to_move.items())
        self._objects_to_move.clear() # Clear buffer since about to be added

        for handle, pose in obj_list:

            obj = self._get_scene_object(handle)
    
            with torch.inference_mode():
                tensor_pose = torch.tensor(
                    [pose[0], pose[1], pose[2],
                    pose[6], pose[3], pose[4], pose[5]],  # qw,qx,qy,qz
                    device=self.env.device, dtype=torch.float32
                ).unsqueeze(0)

                obj.write_root_pose_to_sim(tensor_pose)

        



    def _record_object_poses(self):
        """
        Store world poses of all initialized objects.
        """

        if self.env is None:
            return 

        self._object_poses = {}

        for obj in self._initialized_objects:
            handle = obj.handle
            sim_obj = self._get_scene_object(handle)

    
            # Isaac Sim root pose is [x, y, z, qw, qx, qy, qz]
            root_pose = sim_obj.data.root_state_w[0, :7].cpu().numpy()

            self._object_poses[handle] = np.array([
                root_pose[0], root_pose[1], root_pose[2],  # x, y, z
                root_pose[4], root_pose[5], root_pose[6], root_pose[3],  # qx, qy, qz, qw
            ])


    def _setup_env_cfg(self, args_cli: argparse.Namespace) -> "ManagerBasedEnvCfg":
        """
        (Hook) Creates and configures the environment
        Args:
            args_cli (argparse.Namespace): Command-line arguments parsed by IsaacSession.

        Returns:
            (ManagerBasedEnvCfg): The configuration used to initialize the environment.
        """

        # Must be imported after kit loaded
        from robot_motion_interface.isaacsim.config.bimanual_arm_objects_env_config import BimanualArmObjectEnvCfg
        
        env_cfg = BimanualArmObjectEnvCfg()
        env_cfg.scene.num_envs = args_cli.num_envs
        env_cfg.sim.device = args_cli.device

        return env_cfg

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

        # Load any objects that have poses pending
        self._move_objects()

        # Log poses
        self._record_object_poses()
        



if __name__ == "__main__":

    config_path = Path(__file__).resolve().parents[3] / "config" / "isaacsim_config.yaml"

    isaac = IsaacsimObjectInterface.from_yaml(config_path)
    isaac.start_loop()