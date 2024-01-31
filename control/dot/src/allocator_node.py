#!/usr/bin/env python3
"""
Author: Jason Hughes
Date: May 2023
About: starter script for Optimal Transport Allocator Node in ROS2 
"""
import rclpy
from allocator import DOTAllocator

def main(args=None):
    rclpy.init(args=args)

    allocator = DOTAllocator()

    rclpy.spin(allocator)

    allocator.destroy_node()
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()
