from robot_motion_interface.isaacsim.config.bimanual_arm_env_config import BimanualArmSceneCfg, ActionsCfg, ObservationsCfg, EventCfg  

from isaaclab.envs import ManagerBasedEnvCfg
from isaaclab.assets import RigidObjectCfg

from isaaclab.utils import configclass
import isaaclab.sim as sim_utils


from pathlib import Path


USD_DIR = Path(__file__).resolve().parent.parent / "usds"
    


@configclass
class BimanualArmObjectSceneCfg(BimanualArmSceneCfg):
    """Configuration for the Bimanual Arm with a bunch of objects"""
    

    bowl = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/Bowl",
        spawn=sim_utils.UsdFileCfg(
            usd_path=str(USD_DIR / "bowl" / "bowl.usd"),
            scale=(1.0, 1.0, 1.0),
            rigid_props = sim_utils.RigidBodyPropertiesCfg(rigid_body_enabled=True, kinematic_enabled=False),
            collision_props=sim_utils.CollisionPropertiesCfg(collision_enabled=True),
            visible=False,
        ),
    )

    cup = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/cup",
        spawn=sim_utils.CylinderCfg(
            radius=0.03, height=0.08,
            rigid_props = sim_utils.RigidBodyPropertiesCfg(rigid_body_enabled=True, kinematic_enabled=False),
            collision_props=sim_utils.CollisionPropertiesCfg(collision_enabled=True),
            visible=False,
        ),
    )


    cube = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/Cube",
        spawn=sim_utils.CuboidCfg(
            size=(0.08, 0.06, 0.06),
            mass_props = sim_utils.MassPropertiesCfg(mass=0.05),
            rigid_props = sim_utils.RigidBodyPropertiesCfg(rigid_body_enabled=True, kinematic_enabled=False),
            collision_props=sim_utils.CollisionPropertiesCfg(collision_enabled=True),
            visible=False,
        ),
    )
    
    cylinder = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/cylinder",
        spawn=sim_utils.CylinderCfg(
            radius=0.05, height=0.1,
            rigid_props = sim_utils.RigidBodyPropertiesCfg(rigid_body_enabled=True, kinematic_enabled=False),
            collision_props=sim_utils.CollisionPropertiesCfg(collision_enabled=True),
            visible=False,
        ),
    )

    sphere = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/sphere",
        spawn=sim_utils.SphereCfg(
            radius=0.05,
            rigid_props = sim_utils.RigidBodyPropertiesCfg(rigid_body_enabled=True, kinematic_enabled=False),
            collision_props=sim_utils.CollisionPropertiesCfg(collision_enabled=True),
            visible=False,
        ),
    )


@configclass
class BimanualArmObjectEnvCfg(ManagerBasedEnvCfg):
    """Configuration for the Bimanual Arm environment."""

    scene = BimanualArmObjectSceneCfg(num_envs=1024, env_spacing=2.5)
    observations = ObservationsCfg()
    actions = ActionsCfg()
    events = EventCfg()

    def __post_init__(self):
        """Post initialization."""
        self.viewer.eye = [0.0, 3.0, 1.5]
        self.viewer.lookat = [0.0, 0.0, 1.0]
        self.decimation = 1 
        self.sim.dt = 0.0167 