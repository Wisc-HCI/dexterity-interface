from robot_motion_interface.isaacsim.config.bimanual_arm_env_config import BimanualArmSceneCfg, ActionsCfg, ObservationsCfg, EventCfg  

from isaaclab.envs import ManagerBasedEnvCfg
from isaaclab.assets import RigidObjectCfg, AssetBaseCfg

from isaaclab.utils import configclass
import isaaclab.sim as sim_utils

from pathlib import Path


USD_DIR = Path(__file__).resolve().parent.parent / "usds"
    


@configclass
class BimanualArmObjectSceneCfg(BimanualArmSceneCfg):
    """Configuration for the Bimanual Arm with a bunch of objects"""
    
    bowl_1 = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/bowl_1",
        spawn=sim_utils.UsdFileCfg(
            usd_path=str(USD_DIR / "bowl.usd"),
            scale=(1.0, 1.0, 1.0),
            rigid_props = sim_utils.RigidBodyPropertiesCfg(rigid_body_enabled=True, kinematic_enabled=False),
            visible=False,
        ),
    )


    cup_1 = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/cup_1",
        spawn=sim_utils.UsdFileCfg(
            usd_path=str(USD_DIR / "cup.usd"),
            scale=(1.0, 1.0, 1.0),
            rigid_props = sim_utils.RigidBodyPropertiesCfg(rigid_body_enabled=True, kinematic_enabled=False),
            visible=False,
        ),
    )

    # Isaaclab-3.0.0-beta has breaking changes where usds cannot be shared anymore. This may be fixed
    # when official version is out.
    cup_2 = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/cup_2",
        spawn=sim_utils.UsdFileCfg(
            usd_path=str(USD_DIR / "cup_2.usd"),
            scale=(1.0, 1.0, 1.0),
            rigid_props = sim_utils.RigidBodyPropertiesCfg(rigid_body_enabled=True, kinematic_enabled=False),
            visible=False,
        ),
    )

   
    fork_1 = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/fork_1",
        spawn=sim_utils.UsdFileCfg(
            usd_path=str(USD_DIR / "fork.usd"),
            scale=(1.0, 1.0, 1.0),
            rigid_props = sim_utils.RigidBodyPropertiesCfg(rigid_body_enabled=True, kinematic_enabled=False),
            visible=False,
        ),
    )


    spoon_1 = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/spoon_1",
        spawn=sim_utils.UsdFileCfg(
            usd_path=str(USD_DIR / "spoon.usd"),
            scale=(1.0, 1.0, 1.0),
            rigid_props = sim_utils.RigidBodyPropertiesCfg(rigid_body_enabled=True, kinematic_enabled=False,
                                                           solver_position_iteration_count=16
                                                           ),
            visible=False,
        ),
    )


    bin_1 = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/bin_1",
        spawn=sim_utils.UsdFileCfg(
            usd_path=str(USD_DIR / "bin.usd"),
            scale=(1.0, 1.0, 1.0),
            rigid_props = sim_utils.RigidBodyPropertiesCfg(rigid_body_enabled=True, kinematic_enabled=False),
            visible=False,
        ),
    )

    cube_1 = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/cube_1",
        spawn=sim_utils.CuboidCfg(
            size=(0.08, 0.06, 0.06),
            mass_props = sim_utils.MassPropertiesCfg(mass=0.1),
            rigid_props = sim_utils.RigidBodyPropertiesCfg(rigid_body_enabled=True, kinematic_enabled=False),
            collision_props=sim_utils.CollisionPropertiesCfg(collision_enabled=True),
            physics_material=sim_utils.RigidBodyMaterialCfg(
                static_friction=0.5,
                dynamic_friction=0.4,
                friction_combine_mode="max",
            ),
            visible=False,
        ),
    )

    cylinder_1 = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/cylinder_1",
        spawn=sim_utils.CylinderCfg(
            radius=0.05, height=0.1,
            mass_props = sim_utils.MassPropertiesCfg(mass=0.1),
            rigid_props = sim_utils.RigidBodyPropertiesCfg(rigid_body_enabled=True, kinematic_enabled=False),
            collision_props=sim_utils.CollisionPropertiesCfg(collision_enabled=True),
            physics_material=sim_utils.RigidBodyMaterialCfg(
                static_friction=0.5,
                dynamic_friction=0.4,
                friction_combine_mode="max",
            ),
            visible=False,
        ),
    )

    sphere_1 = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/sphere_1",
        spawn=sim_utils.SphereCfg(
            radius=0.05,
            mass_props = sim_utils.MassPropertiesCfg(mass=0.1),
            rigid_props = sim_utils.RigidBodyPropertiesCfg(rigid_body_enabled=True, kinematic_enabled=False),
            collision_props=sim_utils.CollisionPropertiesCfg(collision_enabled=True),
            physics_material=sim_utils.RigidBodyMaterialCfg(
                static_friction=0.5,
                dynamic_friction=0.4,
                friction_combine_mode="max",
            ),
            visible=False,
        ),
    )


    marker = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/marker",
        spawn=sim_utils.UsdFileCfg(
            usd_path=str(USD_DIR / "marker.usd"),
            scale=(1.0, 1.0, 1.0),
            visible=False,
        ),
    )





@configclass
class BimanualArmObjectEnvCfg(ManagerBasedEnvCfg):
    """Configuration for the Bimanual Arm environment."""

    scene = BimanualArmObjectSceneCfg(num_envs=1, env_spacing=2.5)
    observations = ObservationsCfg()
    actions = ActionsCfg()
    events = EventCfg()

    def __post_init__(self):
        """Post initialization."""
        self.viewer.eye = [0.0, -1.3, 2.5]
        self.viewer.lookat = [0.0, 0.0, 1.2]
        self.decimation = 1
        self.sim.dt = 0.005
        self.sim.render_interval = int(0.02 / self.sim.dt)  # 50 FPS