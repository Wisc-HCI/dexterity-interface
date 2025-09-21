from robot_motion_interface.isaacsim.utils.import_module import import_module
from argparse import ArgumentParser
from types import SimpleNamespace
from isaaclab.app import AppLauncher


class IsaacSession:
    

    def __init__(self, import_paths: list[str], parser: ArgumentParser = None):
        """
        Inits object that owns the Kit app lifecycle and exposes late-imported Isaac modules.
        
        Args:
            import_paths(list[str]): list of string imports. See utils.import_module for
                string formatting.
            parser (ArgumentParser, optional): 
                An existing argument parser to extend. If None, a new parser will be created.
        """
        
        self.import_paths = import_paths
        self.args = None
        self.app = None

        # Late-loaded modules/objects
        self.mods = SimpleNamespace()  # holds resolved imports as attributes


        # PRIVATE
        self._parser = parser

        if not self._parser:
            self._parser = ArgumentParser(description="Isaacsim Session")


    def __enter__(self) -> "IsaacSession":
        """
        Launch IsaacSim kit and app

        Returns:
            (IsaacSession) Object that lets you access app and late-load modules. 
        """

        AppLauncher.add_app_launcher_args(self._parser)
        self.args = self._parser.parse_args()
    
        app_launcher = AppLauncher(self.args)
        self.app = app_launcher.app

        # 2) Now that Kit is up, import Isaac/Kit-dependent modules
        for spec in self.import_paths:
            name, obj = import_module(spec)
            setattr(self.mods, name, obj)

        return self



    def __exit__(self, exc_type, exc, tb) -> None:
        """
        Ensure app closes even on exceptions
        """
        if self.app is not None:
            self.app.close()