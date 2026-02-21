from pathlib import Path
from math import pi

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg

THIS_DIR = Path(__file__).resolve().parent
USD_PATH = (THIS_DIR.parent / "usds" / "bimanual_arms" / "bimanual_arms.usd") 

BIMANUAL_ARM_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=str(USD_PATH),
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            rigid_body_enabled=True,
            enable_gyroscopic_forces=True,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,
            solver_position_iteration_count=4,
            solver_velocity_iteration_count=4,
            sleep_threshold=0.005,
            stabilization_threshold=0.001,
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.0), joint_pos={ 
            r"(left|right)_panda_joint1": 0.0,
            r"(left|right)_panda_joint2": -pi/4,
            r"(left|right)_panda_joint3": 0.0,
            r"(left|right)_panda_joint4": -3 * pi/4,
            r"(left|right)_panda_joint5": 0.0,
            r"(left|right)_panda_joint6": pi/2,
            r"(left|right)_panda_joint7": pi/4,
            # Everything else not listed is 0
        }
    ),
    # These are huge to match response of actual robot despite low sim hertz
    actuators={
        "arm_actuators": ImplicitActuatorCfg(
            joint_names_expr=[r"(left|right)_panda_joint\d"],
            stiffness={
                r"(left|right)_panda_joint1": 5000.0,
                r"(left|right)_panda_joint2": 5000.0,
                r"(left|right)_panda_joint3": 5000.0,
                r"(left|right)_panda_joint4": 5000.0,
                r"(left|right)_panda_joint5": 5000.0,
                r"(left|right)_panda_joint6": 5000.0,
                r"(left|right)_panda_joint7": 5000.0,
            },
            damping={
                r"(left|right)_panda_joint1": 100.0,
                r"(left|right)_panda_joint2": 100.0,
                r"(left|right)_panda_joint3": 100.0,
                r"(left|right)_panda_joint4": 100.0,
                r"(left|right)_panda_joint5": 100.0,
                r"(left|right)_panda_joint6": 100.0,
                r"(left|right)_panda_joint7": 100.0,
            },
            armature=0.001,

            # No limits
            effort_limit_sim=1e6,
            effort_limit=1e6,
            velocity_limit_sim=1e6,
            velocity_limit=1e6,
        ),
        "gripper_actuators": ImplicitActuatorCfg(
            joint_names_expr=[r"(left|right)_F\d+M\d+"],
            stiffness=60.0,
            damping=5.0,
            armature=0.01,
            # No limits
            effort_limit_sim=1e6,
            effort_limit=1e6,
            velocity_limit_sim=1e6,
            velocity_limit=1e6,
            
        ),
    },
)
