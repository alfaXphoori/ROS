#!/usr/bin/env python3
"""Exercise 2: Multi-sensor Webots simulation launch file"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, FindPackageShare
from launch_ros.actions import Node
from pathlib import Path

def generate_launch_description():
    pkg_share = FindPackageShare('ce_robot').find('ce_robot')
    world_path = Path(pkg_share) / 'worlds' / 'sensor_robot_world.wbt'
    
    world_file_arg = DeclareLaunchArgument(
        'world_file',
        default_value=str(world_path),
        description='Path to Webots world file'
    )
    
    webots_process = ExecuteProcess(
        cmd=[
            'webots',
            LaunchConfiguration('world_file'),
            '--batch'
        ],
        output='screen',
        name='webots_simulator'
    )
    
    # Optional: Add ROS 2 sensor visualization node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', str(Path(pkg_share) / 'rviz' / 'sensor_config.rviz')],
        output='screen',
        on_exit=None
    )
    
    return LaunchDescription([
        world_file_arg,
        webots_process,
        rviz_node,
    ])
