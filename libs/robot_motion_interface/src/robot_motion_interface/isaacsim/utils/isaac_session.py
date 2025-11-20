from argparse import ArgumentParser
from isaaclab.app import AppLauncher



class IsaacSession:
    
    def __init__(self, parser: ArgumentParser = None, parser_defaults: dict = None):
        """
        Inits object that owns the Kit app lifecycle and exposes late-imported Isaac modules.
        
        Args:
            parser (ArgumentParser): 
                An existing argument parser to extend. If None, a new parser will be created.
            parser_defaults (dict): Defaults to set for parser. If None, will use Isaac Lab defaults.
        """

        self.args = None
        self.app = None

        # PRIVATE
        self._parser = parser
        self._parser_defaults = parser_defaults
        

        if not self._parser:
            self._parser = ArgumentParser(description="Isaacsim Session")

        



    def __enter__(self) -> "IsaacSession":
        """
        Launch IsaacSim kit and app

        Returns:
            (IsaacSession) Object that lets you access app and late-load modules. 
        """
        
        AppLauncher.add_app_launcher_args(self._parser)
        if self._parser_defaults is not None:
            self._parser.set_defaults(**self._parser_defaults)
            
        self.args = self._parser.parse_args()
        app_launcher = AppLauncher(self.args)
        self.app = app_launcher.app


        return self



    def __exit__(self, exc_type, exc, tb):
        """
        Ensure app closes even on exceptions
        """

        if exc_type:
            raise exc.with_traceback(tb)
        
        if self.app is not None:
            self.app.close()

