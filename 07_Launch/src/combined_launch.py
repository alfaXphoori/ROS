#!/usr/bin/env python3
"""
Exercise 1: Combined Publisher-Subscriber Launch File
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for combined launch"""
    
    publisher_node = Node(
        package='ce_robot',
        executable='hw_status_param_pub',
        name='publisher_ex1',
        output='screen'
    )
    
    subscriber_node = Node(
        package='ce_robot',
        executable='hw_status_callback_pub',
        name='subscriber_ex1',
        output='screen'
    )
    
    return LaunchDescription([
        publisher_node,
        subscriber_node,
    ])
