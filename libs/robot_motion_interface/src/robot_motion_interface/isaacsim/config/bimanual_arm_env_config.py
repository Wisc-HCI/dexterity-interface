from robot_motion_interface.isaacsim.config.bimanual_arm_articulation_config import BIMANUAL_ARM_CFG

import isaaclab.envs.mdp as mdp
from isaaclab.envs import ManagerBasedEnvCfg
from isaaclab.assets import ArticulationCfg, AssetBaseCfg, RigidObjectCfg
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.scene import InteractiveSceneCfg

from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils import configclass
import isaaclab.sim as sim_utils


@configclass
class BimanualArmSceneCfg(InteractiveSceneCfg):
    """Configuration for the Bimanual Arm"""

    ground = AssetBaseCfg(
        prim_path="/World/ground",
        spawn=sim_utils.GroundPlaneCfg(size=(100.0, 100.0)),
    )

    robot: ArticulationCfg = BIMANUAL_ARM_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

    # Need to add table again so collisions happen without needing self-collision for robot
    # which causes jittery mess
    # TODO: perhaps generate this from urdf too
    table = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/Table",
        spawn=sim_utils.CuboidCfg(
            size=(1.8288, 0.62865, 0.045),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                rigid_body_enabled=True,
                kinematic_enabled=True,
            ),
            collision_props=sim_utils.CollisionPropertiesCfg(collision_enabled=True),
        ),
        init_state=RigidObjectCfg.InitialStateCfg(pos=(0.0, 0.0, 0.9144))
    )

    # Ambient Sky
    dome_light = AssetBaseCfg(
        prim_path="/World/DomeLight",
        spawn=sim_utils.DomeLightCfg(
            color=(0.92, 0.95, 1.0),  # cool
            intensity=350.0,
        ),
    )

    # Sun
    sun_light = AssetBaseCfg(
        prim_path="/World/SunLight",
        spawn=sim_utils.DistantLightCfg(
            color=(1.0, 0.97, 0.9),  # warm
            intensity=1200.0,
            angle=1.5,
        ),
    )


@configclass
class ActionsCfg:
    """Action specifications for the environment."""

    joint_efforts = mdp.JointEffortActionCfg(asset_name="robot", joint_names=[".*"])


@configclass
class ObservationsCfg:
    """Observation specifications for the environment."""

    @configclass
    class PolicyCfg(ObsGroup):
        """Observations for policy group."""

        # Order preserved)
        joint_pos_rel = ObsTerm(func=mdp.joint_pos)
        joint_vel_rel = ObsTerm(func=mdp.joint_vel)

        def __post_init__(self) -> None:
            self.enable_corruption = False
            self.concatenate_terms = True

    # observation groups
    policy: PolicyCfg = PolicyCfg()


@configclass
class EventCfg:
    """Configuration for events."""

    reset_position = EventTerm(
        func=mdp.reset_joints_by_offset,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("robot", joint_names=[".*"]),
            "position_range": (0.0, 0.0), # Reset to default position
            "velocity_range": (0.0, 0.0),
        },
    )


@configclass
class BimanualArmEnvConfig(ManagerBasedEnvCfg):
    """Configuration for the Bimanual Arm environment."""

    scene = BimanualArmSceneCfg(num_envs=1024, env_spacing=2.5)
    observations = ObservationsCfg()
    actions = ActionsCfg()
    events = EventCfg()

    def __post_init__(self):
        """Post initialization."""
        self.viewer.eye = [0.0, 3.0, 1.5]
        self.viewer.lookat = [0.0, 0.0, 1.0]
        self.decimation = 1 
        self.sim.dt = 0.008
        self.sim.render_interval = 5