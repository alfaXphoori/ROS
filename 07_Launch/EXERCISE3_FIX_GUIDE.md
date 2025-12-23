# 🔧 Exercise 3 Launch File Fix Guide

## Error Description

```
[ERROR] [launch]: Caught exception in launch (see debug for traceback): 
generate_launch_description.<locals>.handle_battery_exit() missing 1 required positional argument: 'context'
```

## Problem

Your launch file has incomplete event handler functions. They're missing the `context` parameter and/or have incomplete function bodies.

## Solution

Replace your current `~/ros2_ws/src/ce_robot_launch/launch/monitored_system_launch.py` with the complete reference file.

### Option 1: Copy the Reference File (Recommended)

```bash
# On the ROS machine, copy the complete reference file
cp /path/to/07_Launch/src/exercise_3/launch/monitored_system_launch.py \
   ~/ros2_ws/src/ce_robot_launch/launch/monitored_system_launch.py

# Rebuild the package
cd ~/ros2_ws
colcon build --packages-select ce_robot_launch --symlink-install
source install/setup.bash

# Test the fix
ros2 launch ce_robot_launch monitored_system_launch.py simulate_failures:=true
```

### Option 2: Manual Fix

If you can't access the reference file, ensure your event handler functions look exactly like this:

```python
# Event handler: Battery monitor exit with intelligent failure tracking
def handle_battery_exit(event, context):  # ✅ MUST have both event and context
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

# Event handler: CRITICAL navigation controller failure
def handle_navigation_exit(event, context):  # ✅ MUST have both event and context
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
```

## Verification

After fixing, you should see this behavior:

```bash
ros2 launch ce_robot_launch monitored_system_launch.py simulate_failures:=true
```

**Expected output:**
```
[INFO] [launch]: All log files can be found below...
⚠️ Warning: failure_counter.py not found. Using basic monitoring.
[INFO] [launch.user]: ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
[INFO] [launch.user]: 🔍 Production Robot Monitoring System Starting...
✅ Battery Monitor started successfully
✅ Navigation Controller started successfully
✅ Task Processor started successfully

[After ~10 seconds - battery crashes]
[07_battery_monitor-1] [ERROR] 💥 SIMULATED BATTERY MONITOR CRASH!
[INFO] [07_battery_monitor-1]: process has finished cleanly
⚠️ Battery Monitor exited! Attempting restart in 3 seconds...

[3 seconds later]
[INFO] [07_battery_monitor-4]: process started with pid [xxxxx]
[Battery monitor restarts successfully]
```

## Common Mistakes

❌ **Wrong:** `def handle_battery_exit(event):` - Missing context parameter  
❌ **Wrong:** Function has no `return` statement  
❌ **Wrong:** `return` statement is outside the function (indentation error)  
❌ **Wrong:** Incomplete function body (only first few lines copied)

✅ **Correct:** Both functions have `(event, context)` and complete return statements

## Complete Reference File Location

The complete, working reference file is at:
```
07_Launch/src/exercise_3/launch/monitored_system_launch.py
```

This is the master copy that works correctly.
