from robot_motion_interface.interface import Interface
from robot_motion import RobotProperties, JointTorqueController
from robot_motion.ik.multi_chain_ranged_ik import MultiChainRangedIK

import threading
import mujoco
from pathlib import Path
import numpy as np
import yaml
from enum import Enum
import time

import mujoco.viewer


class MujocoControlMode(Enum):
    JOINT_TORQUE = "joint_torque"


class MujocoInterface(Interface):

    def __init__(self, urdf_path:str, ik_settings_path:str, joint_names: list[str], home_joint_positions:np.ndarray,
                base_frame:str, ee_frames:list[str], target_tolerance:float,
                kp: np.ndarray, kd:np.ndarray, max_joint_delta:float, control_mode: MujocoControlMode,
                scene_path: str = None,
                steps_per_render: int = 10):
        """
        Mujoco Interface for running the simulation with accessors for setting
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
            max_joint_delta (float): Caps the joint delta per control step to smooth motion
                toward the setpoint (in radians). If negative (e.g., -1), the limit is ignored.
            control_mode (IsaacsimControlMode): Control mode for the robot (e.g., JOINT_TORQUE).
            scene_path (str): Optional path to a MuJoCo MJCF scene file to attach to the simulation.
            steps_per_render (int): Number of physics steps per viewer render frame.
        """
        super().__init__(joint_names, home_joint_positions, base_frame, ee_frames, target_tolerance)

        self._ik_solver = MultiChainRangedIK(ik_settings_path)
        self._rp = RobotProperties(self._joint_names, urdf_path)

        if control_mode == MujocoControlMode.JOINT_TORQUE:

            self._controller = JointTorqueController(self._rp, kp, kd, gravity_compensation=True, max_joint_delta=max_joint_delta)
        else:
            raise ValueError("Control mode required.")


        self._cur_state = None
        

        # Load Mujoco
        spec = mujoco.MjSpec.from_file(urdf_path)
        spec.compiler.balanceinertia = True
        if scene_path is not None:
            frame = spec.worldbody.add_frame()
            spec.attach(mujoco.MjSpec.from_file(scene_path), frame=frame)
        self._model = spec.compile()
        self._data = mujoco.MjData(self._model)


        # Map joints correctly
        self._joint_qpos_indices = [self._model.joint(name).qposadr[0]
            for name in self._joint_names]
        self._joint_dof_indices = [self._model.joint(name).dofadr[0]
            for name in self._joint_names]
        
        # Start at home so the controller isn't fighting from q=0
        self._data.qpos[self._joint_qpos_indices] = home_joint_positions
        mujoco.mj_forward(self._model, self._data)

        self._steps_per_render = steps_per_render
        self._loop_thread = None
        self._stop_event = threading.Event()



    @classmethod
    def from_yaml(cls, file_path: str):
        """
        Construct an MujocoInterface instance from a YAML configuration file. 
        Note: Any relative paths in the yaml are resolved relative to this package 
        directory (robot_motion_interface).

        Args:
            file_path (str): Path to a YAML file containing keys:
                - "urdf_path" (str): Path to urdf, relative to robot_motion_interface/ (top level).
                - "ik_settings_path" (str): Path to ik settings yaml
                - "joint_names" (list[str]): (n_joints) Ordered list of joint names for the robot.
                - "home_joint_positions" (np.ndarray): (n_joints) Default joint positions (rads)
                - "base_frame" (str): Base frame name for which cartesian poses of end-effector(s) are relative to
                - "ee_frames" (list[str]): (e,) List of frame names for each end-effector
                - "target_tolerance" (float): Threshold (rads) that determines how close the robot's 
                    joints must be to the commanded target to count as reached.
                - "kp" (list[float]): (n_joints) Joint proportional gains.
                - "kd" (list[float]): (n_joints) Joint derivative gains.
                - "max_joint_delta" (float): Caps the joint change per control step
                     to smooth motion toward the setpoint (in radians). If negative (e.g., -1), the limit is ignored.
                - "scene_path" (str): Path to a MuJoCo MJCF scene file, relative to robot_motion_interface.
                - "steps_per_render" (int): Number of physics steps per viewer render frame.
        Returns:
            MujocoInterface: initialized interface
        """
        with open(file_path, "r") as f:
            config = yaml.safe_load(f)

        # Relative file path resolve to package directory, so resolve properly
        pkg_dir = Path(__file__).resolve().parents[3]
        relative_urdf_path = config["urdf_path"]
        urdf_path = str((pkg_dir / relative_urdf_path).resolve())
        relative_ik_settings_path = config["ik_settings_path"]
        ik_settings_path = str((pkg_dir / relative_ik_settings_path).resolve())

        scene_path = None
        if "scene_path" in config:
            scene_path = str((pkg_dir / config["scene_path"]).resolve())

        joint_names = config["joint_names"]
        home_joint_positions = np.array(config["home_joint_positions"], dtype=float)
        base_frame = config["base_frame"]
        ee_frames = config["ee_frames"]
        target_tolerance = config["target_tolerance"]

        kp = np.array(config["kp"], dtype=float)
        kd = np.array(config["kd"], dtype=float)
        max_joint_delta = config["max_joint_delta"]
        control_mode = MujocoControlMode(config["control_mode"])
        steps_per_render = config.get("steps_per_render", 5)

        return cls(urdf_path, ik_settings_path, joint_names, home_joint_positions, base_frame, ee_frames,
                   target_tolerance, kp, kd, max_joint_delta, control_mode, scene_path, steps_per_render)
    

    def set_joint_positions(self, q:np.ndarray, joint_names:list[str] = None, blocking:bool = False):
        """
        Set the controller's target joint positions at selected joints.

        Args:
            q (np.ndarray): (n_joint_names,) Desired joint angles in radians.
            joint_names (list[str]): (n_joint_names,) Names of joints to command in the same
                order as `q`. If None, assumes q is length of all joints.
            blocking (bool): If True, the call should returns only after the controller
                achieves the target. If False, returns after queuing the request.
        """

        q = self._partial_to_full_joint_positions(q, joint_names)
        
        self._controller.set_setpoint(q)
        
        if blocking:
            self._block_until_reached_target()


    def joint_state(self) -> np.ndarray:
        """
        Get the current joint positions and velocities in order of joint_names.

        Returns:
            (np.ndarray): (n_joints * 2) Current joint angles in radians and joint velocities
                in rad/s. [panda_left_pos;  tesollo_left_pos; panda_right_pos; tesollo_right_pos; 
                panda_left_vel; tesollo_left_vel; panda_right_vel; tesollo_right_vel]
        """

        return self._cur_state


    def start_loop(self):
        """

        Start the passive MuJoCo viewer and physics loop in a background thread.

        Opens the MuJoCo native viewer window and runs physics and control steps
        continuously until stop_loop() is called or the viewer window is closed.
        Uses the first camera in the scene if one is present.
        """
        self._stop_event.clear()

        def viewer_thread():
        
            with mujoco.viewer.launch_passive(self._model, self._data) as v:
                # Use first camera in scene (if present)
                if self._model.ncam > 0:
                    v.cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
                    v.cam.fixedcamid = 0 

                while v.is_running() and not self._stop_event.is_set():
                    for _ in range(self._steps_per_render):
                        self._step()
                    v.sync()


        self._loop_thread = threading.Thread(target=viewer_thread, daemon=True)
        self._loop_thread.start()


    def _step(self):
        """

        Advance physics by one step and update the current joint state.

        Reads joint positions and velocities, computes torque efforts via the
        joint torque controller, applies them, and steps the MuJoCo simulation
        forward by one timestep.
        """
        qpos = self._data.qpos[self._joint_qpos_indices]
        qvel = self._data.qvel[self._joint_dof_indices]
        self._cur_state = np.concatenate([qpos, qvel])
        joint_efforts = self._controller.step(self._cur_state)
        self._data.qfrc_applied[self._joint_dof_indices] = joint_efforts
        mujoco.mj_step(self._model, self._data)

    def stop_loop(self):
        """

        Signal the physics loop to stop and block until the thread exits.
        """
        self._stop_event.set()
        self._loop_thread.join()



if __name__ == "__main__":
    import time

    pkg_dir = Path(__file__).resolve().parents[3]
    config_path = pkg_dir / "config" / "mujoco_config.yaml"

    arms = MujocoInterface.from_yaml(config_path)

    try:
        arms.start_loop()
        arms.home()
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nStopping Mujoco.")
    finally:
        arms.stop_loop()
