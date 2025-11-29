#!/usr/bin/env python3
"""
Exercise 2: Multi-Node Launch with Parameters
Launches publisher, subscriber, and server with configured parameters
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description with multiple nodes and parameters"""
    
    publisher_node = Node(
        package='ce_robot',
        executable='hw_status_param_pub',
        name='hw_publisher_ex2',
        output='screen',
        parameters=[
            {'robot_name': 'robot_lab2'},
            {'robot_number': 200},
            {'publish_rate': 2.0},
            {'debug_mode': False},
        ]
    )
    
    subscriber_node = Node(
        package='ce_robot',
        executable='hw_status_callback_pub',
        name='hw_subscriber_ex2',
        output='screen',
        parameters=[
            {'robot_name': 'hw_monitor'},
            {'robot_number': 0},
            {'publish_rate': 2.0},
            {'debug_mode': True},
            {'temperature_offset': 5},
        ]
    )
    
    service_server = Node(
        package='ce_robot',
        executable='cal_rect_server',
        name='rect_server_ex2',
        output='screen'
    )
    
    return LaunchDescription([
        publisher_node,
        subscriber_node,
        service_server,
    ])
