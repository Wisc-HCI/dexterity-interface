"""
Shows manipulating objects in isaacsim

Usage:
   python -m robot_motion_interface.examples.isaacsim_objects 
"""

from robot_motion_interface.isaacsim.isaacsim_object_interface import IsaacsimObjectInterface, Object, ObjectHandle
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
    init_objects = [
        Object(handle=ObjectHandle.CUBE, pose=(-0.259, -0.092, 0.95, 0, 0, 0, 1)),
        Object(handle=ObjectHandle.SPHERE, pose=(0, 0, 0.95, 0, 0, 0, 1)),
        
       
    ]
    next_objects = [
        Object(handle=ObjectHandle.CYLINDER, pose=(0.1, 0.1, 0.95, 0, 0, 0, 1)),
        Object(handle=ObjectHandle.BOWL, pose=(-0.2, 0.25, 0.95, 0, 0, 0, 1))
        
    ]
    
    interface.place_objects(init_objects)
    time.sleep(20)
    interface.place_objects(next_objects)
    time.sleep(5)
    interface.move_object('bowl', (0.2, 0.25, 0.95, 0, 0, 0, 1))


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
