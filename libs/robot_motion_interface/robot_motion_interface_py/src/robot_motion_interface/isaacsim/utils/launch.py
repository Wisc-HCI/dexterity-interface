from argparse import ArgumentParser, Namespace
from isaaclab.app import AppLauncher

def launch(parser: ArgumentParser = None, args_cli:Namespace = None):
    """
    Launches IsaacSim GUI

    Args:
        parser (ArgumentParser, optional): 
            An existing argument parser to extend. If None, a new parser will be created.

    Returns:
        simulation_app: 
            An instance of the IsaacSim application.
        args_cli (Namespace):
            Parsed command-line arguments.
    """
    if not parser:
        parser = ArgumentParser(description="Isaacsim launcher")


    AppLauncher.add_app_launcher_args(parser)
    args_cli = parser.parse_args()
    

    app_launcher = AppLauncher(args_cli)
    simulation_app = app_launcher.app

    return simulation_app, args_cli


if __name__ == "__main__":
    launch()
