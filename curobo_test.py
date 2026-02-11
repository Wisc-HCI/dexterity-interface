# Third Party
import torch
from pathlib import Path


# cuRobo
from curobo.types.math import Pose
from curobo.types.robot import JointState
from curobo.wrap.reacher.motion_gen import MotionGen, MotionGenConfig, MotionGenPlanConfig

world_config = {
    # "mesh": {
    #     "base_scene": {
    #         "pose": [10.5, 0.080, 1.6, 0.043, -0.471, 0.284, 0.834],
    #         "file_path": "curobo/scene/nvblox/srl_ur10_bins.obj",
    #     },
    # },
    "cuboid": {
        "block": {
            "dims": [0.05, 0.05, 0.08],  # x, y, z
            "pose": [0.3, 0.1, 0, 0.0, 0.0, 0.0, 1.0],  # x, y, z, qw, qx, qy, qz
            # [-0.3, 0.1, 0.95, 0.0, 0.0, 0.0, 1.0]
        },
    },
}

robot_cfg_path = Path(__file__).parent / "libs" /"bimanual_arms.yml"

motion_gen_config = MotionGenConfig.load_from_robot_config(
    str(robot_cfg_path),
    world_config,
    interpolation_dt=0.01,
)
motion_gen = MotionGen(motion_gen_config)
motion_gen.warmup()

goal_pose = Pose.from_list([0.3, 0.0, 1.3, 0.0, 0.0, 1.0, 0.0])  # x, y, z, qw, qx, qy, qz
start_state = JointState.from_position(
    torch.tensor([[0.0, -0.7854, 0.0, -2.3562, 0.0, 1.5708, 0.7854]]).cuda(),
    joint_names=[
        "right_panda_joint1",
        "right_panda_joint2",
        "right_panda_joint3",
        "right_panda_joint4",
        "right_panda_joint5",
        "right_panda_joint6",
        "right_panda_joint7",
    ],
)

result = motion_gen.plan_single(start_state, goal_pose, MotionGenPlanConfig(max_attempts=1))
traj = result.get_interpolated_plan()  # result.interpolation_dt has the dt between timesteps
print("Trajectory Generated: ", result.success)

if result.success:
    print(traj.position)

# Left arm
print("\n--- Left Arm ---")
left_cfg_path = Path(__file__).parent / "bimanual_arms_left.yml"

left_motion_gen_config = MotionGenConfig.load_from_robot_config(
    str(left_cfg_path),
    world_config,
    interpolation_dt=0.01,
)
left_motion_gen = MotionGen(left_motion_gen_config)
left_motion_gen.warmup()

left_goal_pose = Pose.from_list([-0.3, 0.0, 1.3, 0.0, 0.0, 1.0, 0.0])  # x, y, z, qw, qx, qy, qz
left_start_state = JointState.from_position(
    torch.tensor([[0.0, -0.7854, 0.0, -2.3562, 0.0, 1.5708, 0.7854]]).cuda(),
    joint_names=[
        "left_panda_joint1",
        "left_panda_joint2",
        "left_panda_joint3",
        "left_panda_joint4",
        "left_panda_joint5",
        "left_panda_joint6",
        "left_panda_joint7",
    ],
)

left_result = left_motion_gen.plan_single(left_start_state, left_goal_pose, MotionGenPlanConfig(max_attempts=1))
left_traj = left_result.get_interpolated_plan()
print("Trajectory Generated: ", left_result.success)

if left_result.success:
    print(left_traj.position)