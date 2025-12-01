#!/usr/bin/env python3
"""Webots-ROS 2 bridge configuration and initialization"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    domain_id_arg = DeclareLaunchArgument(
        'ros_domain_id',
        default_value='0',
        description='ROS Domain ID for DDS communication'
    )
    
    verbosity_arg = DeclareLaunchArgument(
        'verbosity',
        default_value='info',
        description='Logging verbosity level'
    )
    
    # ROS 2 daemon (for topic discovery)
    daemon_process = ExecuteProcess(
        cmd=['ros2', 'daemon', 'start'],
        output='screen',
        on_exit=None
    )
    
    return LaunchDescription([
        domain_id_arg,
        verbosity_arg,
        daemon_process,
    ])
