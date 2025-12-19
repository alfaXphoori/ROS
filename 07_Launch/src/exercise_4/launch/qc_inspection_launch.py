#!/usr/bin/env python3
"""
Exercise 4: Quality Control Inspection System Launch File
Implements event handlers and intelligent failure recovery for manufacturing QC
Real-world scenario: 24/7 production line quality inspection with camera, sensors, and reporting
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
        default_value='vision',
        description='Critical node that triggers shutdown if it fails',
        choices=['vision', 'fusion', 'report', 'none']
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
    
    # Vision Inspector Node - CRITICAL (shutdown on failure)
    vision_inspector_node = Node(
        package='ce_robot',
        executable='07_vision_inspector',
        name='vision_inspector',
        output='screen',
        parameters=[
            {'robot_id': 'QC-VISION-INSPECTOR-001'},
            {'robot_type': 'inspection'},
            {'zone_id': 'FACTORY-QC-STATION-1'},
            {'camera_resolution': '1920x1080'},
            {'inspection_rate_hz': 1.0},
            {'defect_threshold': 0.85},
            {'simulate_failure': simulate_failures},
        ],
    )
    
    # Sensor Fusion Node - HIGH criticality (auto-restart)
    sensor_fusion_node = Node(
        package='ce_robot',
        executable='07_sensor_fusion',
        name='sensor_fusion',
        output='screen',
        parameters=[
            {'robot_id': 'QC-SENSOR-FUSION-001'},
            {'robot_type': 'inspection'},
            {'zone_id': 'FACTORY-QC-STATION-1'},
            {'fusion_rate_hz': 2.0},
            {'sensor_count': 4},
            {'simulate_failure': simulate_failures},
        ],
    )
    
    # Report Generator Node - NON-CRITICAL (log only)
    report_generator_node = Node(
        package='ce_robot',
        executable='07_report_generator',
        name='report_generator',
        output='screen',
        parameters=[
            {'robot_id': 'QC-REPORT-GENERATOR-001'},
            {'robot_type': 'inspection'},
            {'zone_id': 'FACTORY-QC-STATION-1'},
            {'report_rate_hz': 0.5},
            {'report_format': 'JSON'},
            {'simulate_failure': simulate_failures},
        ],
    )
    
    # Event handler: Vision inspector start
    vision_start_handler = RegisterEventHandler(
        OnProcessStart(
            target_action=vision_inspector_node,
            on_start=[
                LogInfo(msg='✅ Vision Inspector started successfully'),
                LogInfo(msg='🎥 Camera system online - quality inspection active'),
            ]
        )
    )
    
    # Event handler: CRITICAL vision inspector failure
    def handle_vision_exit(event, context):
        """Handle critical vision inspector failure - immediate shutdown"""
        if failure_counter:
            node_name = 'vision_inspector'
            failure_counter.log_failure(
                node_name, 
                "CRITICAL: Vision inspection system crashed - QUALITY RISK!"
            )
        
        return [
            LogInfo(msg='❌ CRITICAL: Vision Inspector failed!'),
            LogInfo(msg='🚨 QUALITY ALERT: Defect detection offline'),
            LogInfo(msg='🛑 EMERGENCY STOP: Halting production line'),
            LogInfo(msg='📞 Emergency contact: QC Manager +1-800-QUALITY'),
            LogInfo(msg='⚠️ Defective products may have passed inspection!'),
            EmitEvent(event=Shutdown(reason='Critical quality system failure - vision')),
        ]
    
    vision_exit_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=vision_inspector_node,
            on_exit=[OpaqueFunction(function=handle_vision_exit)]
        )
    )
    
    # Event handler: Sensor fusion start
    fusion_start_handler = RegisterEventHandler(
        OnProcessStart(
            target_action=sensor_fusion_node,
            on_start=[
                LogInfo(msg='✅ Sensor Fusion started successfully'),
                LogInfo(msg='🔬 Multi-sensor analysis active'),
            ]
        )
    )
    
    # Event handler: Sensor fusion exit with intelligent failure tracking
    def handle_fusion_exit(event, context):
        """Handle sensor fusion exit with failure counting and escalation"""
        if failure_counter:
            node_name = 'sensor_fusion'
            count = failure_counter.increment_failure_count(node_name)
            failure_counter.log_failure(node_name, "Sensor fusion system crashed")
            
            if count >= 3:
                return [
                    LogInfo(msg=f'❌ CRITICAL: Sensor fusion failed {count} times! Manual intervention required.'),
                    LogInfo(msg='⚠️ Alerting maintenance team...'),
                    LogInfo(msg='📧 Email sent to: sensors@factory.com'),
                    LogInfo(msg='📱 SMS alert sent to calibration technician'),
                    EmitEvent(event=Shutdown(reason=f'Persistent node failure: {node_name}'))
                ]
            else:
                return [
                    LogInfo(msg=f'⚠️ Sensor Fusion exited! Attempt {count}/3. Restarting in 3 seconds...'),
                    LogInfo(msg=f'📊 Failure logged with sensor tracking ID'),
                    TimerAction(
                        period=3.0,
                        actions=[
                            Node(
                                package='ce_robot',
                                executable='07_sensor_fusion',
                                name='sensor_fusion',
                                output='screen',
                                parameters=[
                                    {'robot_id': f'QC-SENSOR-FUSION-RESTART-{count}'},
                                    {'robot_type': 'inspection'},
                                    {'zone_id': 'FACTORY-RECOVERY'},
                                    {'fusion_rate_hz': 2.0},
                                    {'sensor_count': 4},
                                    {'simulate_failure': False},  # Don't simulate on restart
                                ],
                            ),
                        ]
                    ),
                ]
        else:
            # Fallback without failure counter
            return [
                LogInfo(msg='⚠️ Sensor Fusion exited! Attempting restart in 3 seconds...'),
                TimerAction(
                    period=3.0,
                    actions=[
                        Node(
                            package='ce_robot',
                            executable='08_sensor_fusion',
                            name='sensor_fusion',
                            output='screen',
                            parameters=[
                                {'robot_id': 'QC-SENSOR-FUSION-RESTART'},
                                {'robot_type': 'inspection'},
                                {'fusion_rate_hz': 2.0},
                            ],
                        ),
                    ]
                ),
            ]
    
    fusion_exit_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=sensor_fusion_node,
            on_exit=[OpaqueFunction(function=handle_fusion_exit)]
        )
    )
    
    # Event handler: Report generator lifecycle (non-critical)
    report_start_handler = RegisterEventHandler(
        OnProcessStart(
            target_action=report_generator_node,
            on_start=[
                LogInfo(msg='✅ Report Generator started successfully'),
                LogInfo(msg='📊 Quality reporting active'),
            ]
        )
    )
    
    report_exit_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=report_generator_node,
            on_exit=[
                LogInfo(msg='⚠️ Report Generator exited (non-critical)'),
                LogInfo(msg='📋 Reports suspended - inspection continues'),
                LogInfo(msg='💾 Data buffering to temporary storage'),
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
        LogInfo(msg='🔍 Quality Control Inspection System Starting...'),
        LogInfo(msg=f'📊 Failure tracking: {config_dir}/failure_counter.py'),
        LogInfo(msg='⚙️ Max restart attempts: 3'),
        LogInfo(msg='🚨 Critical nodes: vision_inspector'),
        LogInfo(msg='🔄 Auto-restart enabled: sensor_fusion'),
        LogInfo(msg='📋 Non-critical: report_generator'),
        LogInfo(msg='━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━'),
        
        # Nodes
        vision_inspector_node,
        sensor_fusion_node,
        report_generator_node,
        
        # Event handlers
        vision_start_handler,
        vision_exit_handler,
        fusion_start_handler,
        fusion_exit_handler,
        report_start_handler,
        report_exit_handler,
    ])
