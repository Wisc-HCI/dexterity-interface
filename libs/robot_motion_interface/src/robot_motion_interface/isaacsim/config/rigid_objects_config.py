
from isaaclab.assets import RigidObjectCfg
import isaaclab.sim as sim_utils

cube_cfg =sim_utils.CuboidCfg(
    size=(0.08, 0.06, 0.06),
    mass_props = sim_utils.MassPropertiesCfg(mass=0.1),
    rigid_props = sim_utils.RigidBodyPropertiesCfg(rigid_body_enabled=True, kinematic_enabled=False),
    collision_props=sim_utils.CollisionPropertiesCfg(collision_enabled=True),
    visible=True,
)

cylinder_cfg = sim_utils.CylinderCfg(
    radius=0.05, height=0.1,
    mass_props = sim_utils.MassPropertiesCfg(mass=0.1),
    rigid_props = sim_utils.RigidBodyPropertiesCfg(rigid_body_enabled=True, kinematic_enabled=False),
    collision_props=sim_utils.CollisionPropertiesCfg(collision_enabled=True),
    visible=True
)

sphere_cfg = sim_utils.SphereCfg(
    radius=0.05,
    mass_props = sim_utils.MassPropertiesCfg(mass=0.1),
    rigid_props = sim_utils.RigidBodyPropertiesCfg(rigid_body_enabled=True, kinematic_enabled=False),
    collision_props=sim_utils.CollisionPropertiesCfg(collision_enabled=True),
    visible=True
)