"""
Shows manipulating objects in isaacsim

Usage:
   python -m robot_motion_interface.examples.isaacsim_objects 
"""

from robot_motion_interface.isaacsim.isaacsim_object_interface import IsaacsimObjectInterface, Object, ObjectType
import os
import time
import threading
import argparse
from pathlib import Path

import numpy as np



def manipulate_objects(interface: IsaacsimObjectInterface):
    """
    Demos moving objects

    Args:
        interface (IsaacsimObjectInterface): The interface instance
    """
    cube_1 = Object(size=(0.1, 0.1, 0.05), position=(-0.259, -0.092, 0.95))
    cube_2 = Object(size=(0.1, 0.1, 0.05), position=(0.259, -0.092, 0.95))
    
    interface.place_objects([cube_1])
    time.sleep(30)
    interface.place_objects([cube_2])


def main(parser: argparse.ArgumentParser = None):
    """
    Simple example of using objects in isaacsim.

    Args:
        parser (ArgumentParser): Argument parser to pass to Isaacsim
    """
    config_dir = Path(__file__).resolve().parents[3] / "config"


    config_path = config_dir / "isaacsim_config.yaml"
    interface = IsaacsimObjectInterface.from_yaml(config_path, parser)
   
 
    obj_thread = threading.Thread(target=manipulate_objects, args=(interface, ))
    obj_thread.start()

    interface.home()
    interface.start_loop()


    try: 
        while(True):
            time.sleep(0.1)
    except (KeyboardInterrupt):
        print("\nStopping Interface.")
    finally:
        interface.stop_loop()  



if __name__ == "__main__":
    main()
