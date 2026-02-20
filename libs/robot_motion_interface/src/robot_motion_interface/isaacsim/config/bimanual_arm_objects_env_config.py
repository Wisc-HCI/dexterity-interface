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
        prim_path="{ENV_REGEX_NS}/bowl",
        spawn=sim_utils.UsdFileCfg(
            usd_path=str(USD_DIR / "bowl.usd"),
            scale=(1.0, 1.0, 1.0),
            rigid_props = sim_utils.RigidBodyPropertiesCfg(rigid_body_enabled=True, kinematic_enabled=False),
            visible=False,
        ),
    )

    bowl_1 = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/bowl_1",
        spawn=sim_utils.UsdFileCfg(
            usd_path=str(USD_DIR / "bowl.usd"),
            scale=(1.0, 1.0, 1.0),
            rigid_props = sim_utils.RigidBodyPropertiesCfg(rigid_body_enabled=True, kinematic_enabled=False),
            visible=False,
        ),
    )

    cup = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/cup",
        spawn=sim_utils.UsdFileCfg(
            usd_path=str(USD_DIR / "cup.usd"),
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

    cup_2 = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/cup_2",
        spawn=sim_utils.UsdFileCfg(
            usd_path=str(USD_DIR / "cup.usd"),
            scale=(1.0, 1.0, 1.0),
            rigid_props = sim_utils.RigidBodyPropertiesCfg(rigid_body_enabled=True, kinematic_enabled=False),
            visible=False,
        ),
    )

    fork = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/fork",
        spawn=sim_utils.UsdFileCfg(
            usd_path=str(USD_DIR / "fork.usd"),
            scale=(1.0, 1.0, 1.0),
            rigid_props = sim_utils.RigidBodyPropertiesCfg(rigid_body_enabled=True, kinematic_enabled=False),
            visible=False,
        ),
    )

    spoon = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/spoon",
        spawn=sim_utils.UsdFileCfg(
            usd_path=str(USD_DIR / "spoon.usd"),
            scale=(1.0, 1.0, 1.0),
            rigid_props = sim_utils.RigidBodyPropertiesCfg(rigid_body_enabled=True, kinematic_enabled=False),
            visible=False,
        ),
    )

    funnel = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/funnel",
        spawn=sim_utils.UsdFileCfg(
            usd_path=str(USD_DIR / "funnel.usd"),
            scale=(1.0, 1.0, 1.0),
            rigid_props = sim_utils.RigidBodyPropertiesCfg(rigid_body_enabled=True, kinematic_enabled=False),
            visible=False,
        ),
    )

    # plastic_box = RigidObjectCfg(
    #     prim_path="{ENV_REGEX_NS}/plastic_box",
    #     spawn=sim_utils.UsdFileCfg(
    #         usd_path=str(USD_DIR / "plastic_box.usd"),
    #         scale=(1.0, 1.0, 1.0),
    #         rigid_props = sim_utils.RigidBodyPropertiesCfg(rigid_body_enabled=True, kinematic_enabled=False),
    #         visible=False,
    #     ),
    # )

    cube = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/cube",
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

    cylinder = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/cylinder",
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

    sphere = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/sphere",
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

    barrier =  RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/barrier",
        spawn=sim_utils.CuboidCfg(
            size=(0.08, 0.5, 0.5),
            mass_props = sim_utils.MassPropertiesCfg(mass=0.1),
            rigid_props = sim_utils.RigidBodyPropertiesCfg(rigid_body_enabled=True, kinematic_enabled=False),
            collision_props=sim_utils.CollisionPropertiesCfg(collision_enabled=True),
            visible=False,
        ),
    )

####################### Many Object Generation ####################### 
cube_spawn_cfg = sim_utils.CuboidCfg(
            size=(0.01, 0.01, 0.01),
            mass_props = sim_utils.MassPropertiesCfg(mass=0.01),
            rigid_props = sim_utils.RigidBodyPropertiesCfg(rigid_body_enabled=True, kinematic_enabled=False),
            collision_props=sim_utils.CollisionPropertiesCfg(collision_enabled=True),
            visual_material=sim_utils.PreviewSurfaceCfg(
                diffuse_color=(0.0, 0.0, 1.0)
            ),
            visible=False,
        )

NUM_CUBES = 50

for i in range(NUM_CUBES):
    setattr(
        BimanualArmObjectSceneCfg,
        f"cube_{i}",
        RigidObjectCfg(
            prim_path=f"{{ENV_REGEX_NS}}/cube_{i}",
            spawn=cube_spawn_cfg,
            init_state=RigidObjectCfg.InitialStateCfg(
                pos=(0.01 * i, 0.0, 0.05),
            ),
        ),
    )

######################################################################

@configclass
class BimanualArmObjectEnvCfg(ManagerBasedEnvCfg):
    """Configuration for the Bimanual Arm environment."""

    scene = BimanualArmObjectSceneCfg(num_envs=1024, env_spacing=2.5)
    observations = ObservationsCfg()
    actions = ActionsCfg()
    events = EventCfg()

    def __post_init__(self):
        """Post initialization."""
        self.viewer.eye = [0.0, -3.0, 1.5]
        self.viewer.lookat = [0.0, 0.0, 1.0]
        self.decimation = 1
        self.sim.dt = 0.005
        self.sim.render_interval = 0.02 / self.sim.dt  # 50 FPS