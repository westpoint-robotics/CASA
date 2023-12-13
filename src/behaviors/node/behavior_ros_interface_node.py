#!/usr/bin/env python3
"""
Author: Jason Hughes
Date: July 2023
About: ros 2 node for behavior interfacing
"""
import os
import sys
import rclpy
import importlib

    
def main(argv):
    
    file_name = argv[0]
    class_name = argv[1]
    
    rclpy.init()
    
    Behavior = getattr(importlib.import_module(file_name), class_name)
    b = Behavior()
    rclpy.spin(b)

    behavior.destroy_node()
    rclpy.shutdown()
    

if __name__ == '__main__':
    main(sys.argv[1:])
