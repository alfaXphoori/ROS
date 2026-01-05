#!/usr/bin/env python3
"""
Exercise 4: YAML Configuration and Composition
Loads parameters from YAML files and composes other launch files
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    LogInfo
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description with YAML config"""
    
    # Get package directory - using os.path for compatibility
    current_dir = os.path.dirname(os.path.abspath(__file__))
    config_dir = os.path.join(current_dir, 'config')
    
    # Declare arguments
    robot_config_arg = DeclareLaunchArgument(
        'robot_config',
        default_value='small',
        description='Robot configuration to load (small, large, simulation, or hardware)',
        choices=['small', 'large', 'simulation', 'hardware']
    )
    
    include_simple_launch_arg = DeclareLaunchArgument(
        'include_simple_launch',
        default_value='false',
        description='Include the simple_launch.py as well'
    )
    
    # Get configurations
    robot_config = LaunchConfiguration('robot_config')
    include_simple = LaunchConfiguration('include_simple_launch')
    
    # Build YAML file path - direct path construction
    config_file = PathJoinSubstitution([
        config_dir,
        ['robot_', robot_config, '.yaml']
    ])
    
    # Set environment variable (optional, but useful for debugging)
    set_env = SetEnvironmentVariable(
        'ROS_DOMAIN_ID', '42'
    )
    
    # Load nodes with YAML parameters
    # Using nodes from 07_Launch exercises
    battery_monitor_node = Node(
        package='ce_robot',
        executable='07_battery_monitor',
        name='battery_monitor',
        output='screen',
        parameters=[config_file]
    )
    
    navigation_node = Node(
        package='ce_robot',
        executable='07_navigation_controller',
        name='navigation_controller',
        output='screen',
        parameters=[config_file]
    )
    
    task_processor_node = Node(
        package='ce_robot',
        executable='07_task_processor',
        name='task_processor',
        output='screen',
        parameters=[config_file]
    )
    
    fleet_monitor_node = Node(
        package='ce_robot',
        executable='07_fleet_monitor',
        name='fleet_monitor',
        output='screen',
        parameters=[config_file]
    )
    
    # Try to include another launch file conditionally
    # Note: This requires simple_launch.py to exist in the same directory
    try:
        pkg_share = FindPackageShare('ce_robot_launch').find('ce_robot_launch')
        simple_launch_include = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    pkg_share,
                    'launch',
                    'simple_launch.py'
                ])
            ]),
            condition=IfCondition(include_simple)
        )
        has_simple_launch = True
    except:
        # If simple_launch.py doesn't exist, create a dummy action
        simple_launch_include = LogInfo(
            msg='Note: simple_launch.py not found - skipping composition',
            condition=IfCondition(include_simple)
        )
        has_simple_launch = False
    
    return LaunchDescription([  # type: ignore
        # Environment
        set_env,
        
        # Arguments
        robot_config_arg,
        include_simple_launch_arg,
        
        # Startup messages
        LogInfo(msg='━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━'),
        LogInfo(msg='📋 YAML Configuration Launch - Exercise 4'),
        LogInfo(msg=['🔧 Loading config: ', config_file]),
        LogInfo(msg='━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━'),
        
        # Nodes with YAML config
        battery_monitor_node,
        navigation_node,
        task_processor_node,
        fleet_monitor_node,
        
        # Composed launch file
        simple_launch_include,
    ])