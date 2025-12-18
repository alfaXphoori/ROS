#!/usr/bin/env python3
"""
Exercise 1: Conditional Launch
Launches nodes based on boolean conditions
Real-world scenario: Flexible warehouse robot configuration
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description with conditional nodes for realistic robot deployment"""
    
    # Declare arguments
    enable_publisher_arg = DeclareLaunchArgument(
        'enable_publisher',
        default_value='true',
        description='Enable robot tag publisher (battery monitor & position tracking)',
        choices=['true', 'false']
    )
    
    enable_service_arg = DeclareLaunchArgument(
        'enable_service',
        default_value='true',
        description='Enable rectangle calculation service (navigation path planning)',
        choices=['true', 'false']
    )
    
    enable_action_arg = DeclareLaunchArgument(
        'enable_action',
        default_value='false',
        description='Enable count action server (task queue management)',
        choices=['true', 'false']
    )
    
    robot_mode_arg = DeclareLaunchArgument(
        'robot_mode',
        default_value='production',
        description='Robot operation mode: production (stable), development (debug), simulation (test)',
        choices=['production', 'development', 'simulation']
    )
    
    robot_type_arg = DeclareLaunchArgument(
        'robot_type',
        default_value='transport',
        description='Robot type: transport (heavy duty), picker (light duty), sorter (medium duty)',
        choices=['transport', 'picker', 'sorter']
    )
    
    # Get configurations
    enable_publisher = LaunchConfiguration('enable_publisher')
    enable_service = LaunchConfiguration('enable_service')
    enable_action = LaunchConfiguration('enable_action')
    robot_mode = LaunchConfiguration('robot_mode')
    robot_type = LaunchConfiguration('robot_type')
    
    # Conditional publisher - Realistic warehouse robot with battery monitoring
    # This node publishes robot status including position, battery, task info
    publisher_node = Node(
        package='ce_robot',
        executable='05_robot_tag_param',
        name='robot_tag_publisher',
        output='screen',
        parameters=[
            {'robot_id': 'AMR-WH-A-001'},  # Autonomous Mobile Robot, Warehouse A, Unit 001
            {'robot_type': robot_type},  # transport, picker, or sorter
            {'zone_id': 'WAREHOUSE-A-DOCK-3'},
            {'fleet_number': 1},
            {'tag_publish_rate': 2.0},  # Publish robot status at 2 Hz
            {'max_payload_kg': 500.0},  # Maximum cargo capacity
            {'current_location': 'DOCK-3-BAY-12'},
            {'assigned_task': 'TRANSPORT-TO-ZONE-B'},
            {'priority_level': 7},  # High priority (1-10 scale)
            {'battery_level': 85.0},  # Current battery percentage
            {'operation_hours': 142.5},  # Total hours in service
        ],
        condition=IfCondition(enable_publisher)
    )
    
    # Conditional service - Navigation path planning (launches only if enabled)
    # Real-world: Calculate safe paths around obstacles with safety margins
    service_node = Node(
        package='ce_robot',
        executable='07_navigate_service',
        name='navigation_service',
        output='screen',
        parameters=[
            {'robot_id': 'AMR-WH-A-001'},
            {'robot_type': robot_type},
            {'zone_id': 'WAREHOUSE-A-DOCK-3'},
            {'max_payload_kg': 500.0},
            {'safety_margin_m': 0.5},
        ],
        condition=IfCondition(enable_service)
    )
    
    # Conditional action - Order picking task queue (launches only if enabled)
    # Real-world: Process warehouse orders with battery and weight management
    action_node = Node(
        package='ce_robot',
        executable='07_task_queue_action',
        name='task_queue_action',
        output='screen',
        parameters=[
            {'robot_id': 'AMR-WH-A-001'},
            {'robot_type': 'picker'},  # Picker optimized for order fulfillment
            {'zone_id': 'WAREHOUSE-A-PICKING-ZONE'},
            {'max_items_per_trip': 50},
            {'battery_level': 100.0},
            {'picking_speed_items_per_min': 12.0},
        ],
        condition=IfCondition(enable_action)
    )
    
    # Debug monitor - Launches UNLESS in production mode
    # Real-world: System diagnostics with CPU, memory, disk monitoring during development
    debug_node = Node(
        package='ce_robot',
        executable='07_debug_monitor',
        name='debug_monitor',
        output='screen',
        parameters=[
            {'robot_id': 'AMR-DEV-001'},
            {'robot_type': 'development'},
            {'zone_id': 'TEST-LAB'},
            {'diagnostic_rate_hz': 0.2},  # Every 5 seconds
            {'enable_network_check': True},
            {'enable_ros_diagnostics': True},
        ],
        condition=UnlessCondition(
            PythonExpression(["'", robot_mode, "' == 'production'"])
        )
    )
    
    # Startup log with mode information
    startup_log = LogInfo(
        msg=['🚀 Starting robot system in [', robot_mode, '] mode with type [', robot_type, ']'],
    )
    
    return LaunchDescription([
        # Arguments
        enable_publisher_arg,
        enable_service_arg,
        enable_action_arg,
        robot_mode_arg,
        robot_type_arg,
        
        # Startup log
        startup_log,
        
        # Conditional nodes
        publisher_node,
        service_node,
        action_node,
        debug_node,
    ])
