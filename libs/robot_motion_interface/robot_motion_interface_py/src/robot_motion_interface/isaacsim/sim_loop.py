from __future__ import annotations

"""
python3 libs/robot_motion_interface/robot_motion_interface_py/src/robot_motion_interface/isaacsim/sim_loop.py --num_envs 1
"""


from robot_motion_interface.isaacsim.utils.isaac_session import IsaacSession

import argparse
import torch

from typing import TYPE_CHECKING

# Imports that need to be loaded after IsaacSession initialized
IMPORTS = [
    "from robot_motion_interface.isaacsim.config.bimanual_arm_env_config import CartpoleEnvCfg",
    "from isaaclab.envs import ManagerBasedEnv"
]

# This is for type checking
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from robot_motion_interface.isaacsim.config.bimanual_arm_env_config import CartpoleEnvCfg
    from isaaclab.envs import ManagerBasedEnv




def main(args_cli, simulation_app):
    """Main function."""
    # parse the arguments
    env_cfg = CartpoleEnvCfg()
    env_cfg.scene.num_envs = args_cli.num_envs
    env_cfg.sim.device = args_cli.device
    # setup base environment
    env = ManagerBasedEnv(cfg=env_cfg)

    # simulate physics
    count = 0

    while simulation_app.is_running():

        with torch.inference_mode():
            # reset
            if count % 300 == 0:
                count = 0
                env.reset()
                print("-" * 80)
                print("[INFO]: Resetting environment...")
            # sample random actions
            joint_efforts = torch.randn_like(env.action_manager.action)
            # step the environment
            obs, _ = env.step(joint_efforts)
            # print current orientation of pole
            print("OBSERVATION", obs["policy"][0][1].item())
            # update counter
            count += 1

    # close the environment
    env.close()


if __name__ == "__main__":

    # add argparse arguments
    parser = argparse.ArgumentParser(description="Tutorial on creating a cartpole base environment.")
    parser.add_argument("--num_envs", type=int, default=16, help="Number of environments to spawn.")

    with IsaacSession(IMPORTS, parser) as sess:

        args_cli = sess.args
        simulation_app = sess.app

        main(args_cli, simulation_app)
