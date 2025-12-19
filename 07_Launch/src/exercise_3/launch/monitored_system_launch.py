#!/usr/bin/env python3
"""
Exercise 3: Event Handlers and Production Monitoring Launch File
Implements node failure detection and automatic restart with production-grade failure tracking
Real-world scenario: 24/7 warehouse robot with intelligent failure recovery
"""

import os
import sys
from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    LogInfo,
    RegisterEventHandler,
    EmitEvent,
    TimerAction,
    OpaqueFunction
)
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Import the failure counter (add config directory to path)
config_dir = Path(__file__).parent.parent / 'config'
sys.path.insert(0, str(config_dir))

try:
    from failure_counter import FailureCounter
except ImportError:
    print("⚠️ Warning: failure_counter.py not found. Using basic monitoring.")
    FailureCounter = None


def generate_launch_description():
    """Generate launch description with event monitoring and failure tracking"""
    
    # Initialize failure counter for production monitoring
    failure_counter = FailureCounter() if FailureCounter else None
    
    # Arguments
    enable_auto_restart_arg = DeclareLaunchArgument(
        'enable_auto_restart',
        default_value='true',
        description='Enable automatic node restart on failure'
    )
    
    critical_node_arg = DeclareLaunchArgument(
        'critical_node',
        default_value='navigation',
        description='Critical node that triggers shutdown if it fails',
        choices=['battery', 'navigation', 'task', 'none']
    )
    
    max_restart_attempts_arg = DeclareLaunchArgument(
        'max_restart_attempts',
        default_value='3',
        description='Maximum restart attempts before escalation'
    )
    
    simulate_failures_arg = DeclareLaunchArgument(
        'simulate_failures',
        default_value='false',
        description='Enable simulated failures for testing event handlers',
        choices=['true', 'false']
    )
    
    # Get configurations
    enable_auto_restart = LaunchConfiguration('enable_auto_restart')
    critical_node = LaunchConfiguration('critical_node')
    max_restart_attempts = LaunchConfiguration('max_restart_attempts')
    simulate_failures = LaunchConfiguration('simulate_failures')
    
    # Battery Monitor Node - HIGH criticality (auto-restart)
    battery_monitor_node = Node(
        package='ce_robot',
        executable='07_battery_monitor',
        name='battery_monitor',
        output='screen',
        parameters=[
            {'robot_id': 'AMR-BATTERY-MONITOR-001'},
            {'robot_type': 'transport'},
            {'zone_id': 'WAREHOUSE-MAIN-FLOOR'},
            {'battery_capacity_ah': 100.0},
            {'battery_voltage_nominal': 48.0},
            {'low_battery_threshold': 20.0},
            {'critical_battery_threshold': 10.0},
            {'monitor_rate_hz': 2.0},
            {'simulate_failure': simulate_failures},
        ],
    )
    
    # Navigation Controller Node - CRITICAL (shutdown on failure)
    navigation_controller_node = Node(
        package='ce_robot',
        executable='07_navigation_controller',
        name='navigation_controller',
        output='screen',
        parameters=[
            {'robot_id': 'AMR-NAV-CONTROLLER-001'},
            {'robot_type': 'transport'},
            {'zone_id': 'WAREHOUSE-MAIN-FLOOR'},
            {'max_speed_ms': 2.5},
            {'safety_radius_m': 0.75},
            {'lidar_range_m': 20.0},
            {'status_rate_hz': 1.0},
            {'simulate_failure': simulate_failures},
        ],
    )
    
    # Task Processor Node - NON-CRITICAL (log only)
    task_processor_node = Node(
        package='ce_robot',
        executable='07_task_processor',
        name='task_processor',
        output='screen',
        parameters=[
            {'robot_id': 'AMR-TASK-PROCESSOR-001'},
            {'robot_type': 'picker'},
            {'zone_id': 'WAREHOUSE-PICKING-AREA'},
            {'max_tasks_per_hour': 50},
            {'process_rate_hz': 0.5},
            {'simulate_failure': simulate_failures},
        ],
    )
    
    # Event handler: Log when battery monitor starts
    battery_start_handler = RegisterEventHandler(
        OnProcessStart(
            target_action=battery_monitor_node,
            on_start=[
                LogInfo(msg='✅ Battery Monitor started successfully'),
            ]
        )
    )
    
    # Event handler: Battery monitor exit with intelligent failure tracking
    def handle_battery_exit(event, context):
        """Handle battery monitor exit with failure counting and escalation"""
        if failure_counter:
            node_name = 'battery_monitor'
            count = failure_counter.increment_failure_count(node_name)
            failure_counter.log_failure(node_name, "Battery monitoring system crashed")
            
            if count >= 3:
                return [
                    LogInfo(msg=f'❌ CRITICAL: Battery monitor failed {count} times! Manual intervention required.'),
                    LogInfo(msg='⚠️ Alerting maintenance team...'),
                    LogInfo(msg='📧 Email sent to: maintenance@warehouse.com'),
                    LogInfo(msg='📱 SMS alert sent to on-call engineer'),
                    EmitEvent(event=Shutdown(reason=f'Persistent node failure: {node_name}'))
                ]
            else:
                return [
                    LogInfo(msg=f'⚠️ Battery Monitor exited! Attempt {count}/3. Restarting in 3 seconds...'),
                    LogInfo(msg=f'📊 Failure logged with robot tracking ID'),
                    TimerAction(
                        period=3.0,
                        actions=[
                            Node(
                                package='ce_robot',
                                executable='07_battery_monitor',
                                name='battery_monitor',
                                output='screen',
                                parameters=[
                                    {'robot_id': f'AMR-BATTERY-MONITOR-RESTART-{count}'},
                                    {'robot_type': 'transport'},
                                    {'zone_id': 'WAREHOUSE-RECOVERY'},
                                    {'battery_capacity_ah': 100.0},
                                    {'monitor_rate_hz': 2.0},
                                    {'simulate_failure': False},  # Don't simulate on restart
                                ],
                            ),
                        ]
                    ),
                ]
        else:
            # Fallback without failure counter
            return [
                LogInfo(msg='⚠️ Battery Monitor exited! Attempting restart in 3 seconds...'),
                TimerAction(
                    period=3.0,
                    actions=[
                        Node(
                            package='ce_robot',
                            executable='07_battery_monitor',
                            name='battery_monitor',
                            output='screen',
                            parameters=[
                                {'robot_id': 'AMR-BATTERY-MONITOR-RESTART'},
                                {'robot_type': 'transport'},
                                {'monitor_rate_hz': 2.0},
                            ],
                        ),
                    ]
                ),
            ]
    
    battery_exit_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=battery_monitor_node,
            on_exit=[OpaqueFunction(function=handle_battery_exit)]
        )
    )
    
    # Event handler: Navigation controller start
    navigation_start_handler = RegisterEventHandler(
        OnProcessStart(
            target_action=navigation_controller_node,
            on_start=[
                LogInfo(msg='✅ Navigation Controller started successfully'),
                LogInfo(msg='🗺️ Safety systems online - collision avoidance active'),
            ]
        )
    )
    
    # Event handler: CRITICAL navigation controller failure
    def handle_navigation_exit(event, context):
        """Handle critical navigation controller failure - immediate shutdown"""
        if failure_counter:
            node_name = 'navigation_controller'
            failure_counter.log_failure(
                node_name, 
                "CRITICAL: Navigation controller crashed - SAFETY RISK!"
            )
        
        return [
            LogInfo(msg='❌ CRITICAL: Navigation Controller failed!'),
            LogInfo(msg='🚨 SAFETY ALERT: Collision avoidance compromised'),
            LogInfo(msg='🛑 EMERGENCY STOP: Shutting down all robot systems'),
            LogInfo(msg='📞 Emergency contact: Safety Team +1-800-ROBOT-911'),
            EmitEvent(event=Shutdown(reason='Critical safety system failure - navigation')),
        ]
    
    navigation_exit_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=navigation_controller_node,
            on_exit=[OpaqueFunction(function=handle_navigation_exit)]
        )
    )
    
    # Event handler: Task processor lifecycle (non-critical)
    task_start_handler = RegisterEventHandler(
        OnProcessStart(
            target_action=task_processor_node,
            on_start=[
                LogInfo(msg='✅ Task Processor started successfully'),
            ]
        )
    )
    
    task_exit_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=task_processor_node,
            on_exit=[
                LogInfo(msg='⚠️ Task Processor exited (non-critical)'),
                LogInfo(msg='📋 Order processing paused - robot continues safety operations'),
            ]
        )
    )
    
    return LaunchDescription([
        # Arguments
        enable_auto_restart_arg,
        critical_node_arg,
        max_restart_attempts_arg,
        simulate_failures_arg,
        
        # Startup messages
        LogInfo(msg='━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━'),
        LogInfo(msg='🔍 Production Robot Monitoring System Starting...'),
        LogInfo(msg=f'📊 Failure tracking: {config_dir}/failure_counter.py'),
        LogInfo(msg='⚙️ Max restart attempts: 3'),
        LogInfo(msg='🚨 Critical nodes: navigation_controller'),
        LogInfo(msg='🔄 Auto-restart enabled: battery_monitor'),
        LogInfo(msg='📋 Non-critical: task_processor'),
        LogInfo(msg='━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━'),
        
        # Nodes
        battery_monitor_node,
        navigation_controller_node,
        task_processor_node,
        
        # Event handlers
        battery_start_handler,
        battery_exit_handler,
        navigation_start_handler,
        navigation_exit_handler,
        task_start_handler,
        task_exit_handler,
    ])
