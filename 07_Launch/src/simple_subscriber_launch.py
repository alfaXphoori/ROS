#!/usr/bin/env python3
"""
Exercise 1: Basic Single-Node Subscriber Launch File
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for basic subscriber"""
    
    subscriber_node = Node(
        package='ce_robot',
        executable='hw_status_callback_pub',
        name='subscriber_ex1',
        output='screen'
    )
    
    return LaunchDescription([
        subscriber_node,
    ])
