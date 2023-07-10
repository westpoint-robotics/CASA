#!/usr/bin/env python3
"""
Author: Jason Hughes
Date: July 2023
About: ros 2 node for behavior interfacing
"""
import os
import yaml
import rclpy
import importlib
from ament_index_python.packages import get_package_share_directory


def loadBehaviors(path):
    with open(path, "r") as f:
        behavior_dict = yaml.safe_load(f)

    return behavior_dict
        
def main(args=None):

    path = os.path.join(
        get_package_share_directory('behaviors'),
        'config',
        'behaviors.yaml'
        )
    
    behavior_dict = loadBehaviors(path)
    behavior_name = input("What behavior do you want to run: ")

    file_name = behavior_dict[behavior_name]["filename"]
    class_name = behavior_dict[behavior_name]["class"]
    
    rclpy.init(args=args)
    
    Behavior = getattr(importlib.import_module(file_name), class_name)
    b = Behavior()
    rclpy.spin(b)

    behavior.destroy_node()
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()
