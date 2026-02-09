#!/usr/bin/env python3
"""
Robot Status Publisher
Simulates realistic autonomous mobile robot with complete telemetry
"""

import rclpy
from rclpy.node import Node
from ce_robot_interfaces.msg import RobotStatus
from std_msgs.msg import Header
import random
import math
import time


class RobotStatusPublisher(Node):
    def __init__(self):
        super().__init__('robot_status_publisher')
        self.publisher_ = self.create_publisher(RobotStatus, 'robot_status', 10)
        self.timer = self.create_timer(1.0, self.publish_status)
        
        # Robot state
        self.robot_id = "AMR-001"
        self.robot_name = "Warehouse Bot Alpha"
        
        # Position and movement
        self.position_x = 0.0
        self.position_y = 0.0
        self.orientation_z = 0.0
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        
        # Waypoints for movement simulation
        self.waypoints = [
            (0.0, 0.0), (5.0, 0.0), (5.0, 5.0), (0.0, 5.0), (0.0, 0.0)
        ]
        self.current_waypoint_idx = 0
        
        # Battery state
        self.battery_voltage = 24.0
        self.battery_percentage = 100
        self.is_charging = False
        
        # Mission state
        self.missions = ["PICKING", "DELIVERING", "RETURNING", "IDLE"]
        self.current_mission = "IDLE"
        self.mission_progress = 0
        self.mission_status = "IN_PROGRESS"
        
        # Safety
        self.emergency_stop = False
        self.obstacle_detected = False
        self.obstacle_distance = 10.0
        
        # System health
        self.cpu_usage = 25.0
        self.temperature = 35.0
        self.motors_enabled = True
        self.health_status = "HEALTHY"
        
        self.get_logger().info(f'🤖 {self.robot_name} ({self.robot_id}) started!')

    def move_towards_waypoint(self):
        """Simulate robot movement towards current waypoint"""
        if self.emergency_stop or not self.motors_enabled:
            self.linear_velocity = 0.0
            self.angular_velocity = 0.0
            return
        
        target_x, target_y = self.waypoints[self.current_waypoint_idx]
        
        # Calculate distance and angle to target
        dx = target_x - self.position_x
        dy = target_y - self.position_y
        distance = math.sqrt(dx**2 + dy**2)
        target_angle = math.atan2(dy, dx)
        
        # If reached waypoint, move to next
        if distance < 0.5:
            self.current_waypoint_idx = (self.current_waypoint_idx + 1) % len(self.waypoints)
            self.mission_progress = int((self.current_waypoint_idx / len(self.waypoints)) * 100)
            
            if self.current_waypoint_idx == 0:
                self.current_mission = random.choice(self.missions)
                self.mission_progress = 0
            return
        
        # Move towards waypoint
        self.linear_velocity = min(1.0, distance * 0.5)  # Max 1 m/s
        
        angle_diff = target_angle - self.orientation_z
        # Normalize angle to [-pi, pi]
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        
        self.angular_velocity = angle_diff * 0.5  # Proportional control
        
        # Update position
        dt = 1.0  # 1 second update
        self.position_x += self.linear_velocity * math.cos(self.orientation_z) * dt
        self.position_y += self.linear_velocity * math.sin(self.orientation_z) * dt
        self.orientation_z += self.angular_velocity * dt

    def update_battery(self):
        """Simulate battery discharge and charging"""
        if self.is_charging:
            # Charging
            self.battery_percentage = min(100, self.battery_percentage + 2)
            self.battery_voltage = 22.0 + (self.battery_percentage / 100.0) * 3.0
            if self.battery_percentage >= 95:
                self.is_charging = False
                self.motors_enabled = True
        else:
            # Discharging (faster when moving)
            discharge_rate = 0.5 if self.linear_velocity > 0 else 0.1
            self.battery_percentage = max(0, self.battery_percentage - discharge_rate)
            self.battery_voltage = 22.0 + (self.battery_percentage / 100.0) * 3.0
            
            # Auto-charge when low
            if self.battery_percentage <= 20:
                self.is_charging = True
                self.motors_enabled = False
                self.current_mission = "CHARGING"
        
        # Calculate time remaining
        discharge_rate = 0.5 if not self.is_charging else -2.0
        if discharge_rate > 0:
            self.battery_time_remaining = int(self.battery_percentage / discharge_rate)
        else:
            self.battery_time_remaining = int((100 - self.battery_percentage) / abs(discharge_rate))

    def update_safety(self):
        """Simulate obstacle detection and safety monitoring"""
        # Random obstacle detection
        if random.random() < 0.1:
            self.obstacle_detected = True
            self.obstacle_distance = random.uniform(0.5, 3.0)
            if self.obstacle_distance < 1.0:
                self.emergency_stop = True
        else:
            self.obstacle_detected = False
            self.obstacle_distance = 10.0
            if self.emergency_stop and random.random() < 0.3:
                self.emergency_stop = False

    def update_health(self):
        """Update system health status"""
        # Simulate CPU usage based on activity
        base_cpu = 25.0
        movement_cpu = abs(self.linear_velocity) * 15.0
        self.cpu_usage = base_cpu + movement_cpu + random.uniform(-5, 5)
        
        # Simulate temperature
        self.temperature = 35.0 + (self.cpu_usage / 100.0) * 15.0 + random.uniform(-2, 2)
        
        # Determine overall health
        if self.emergency_stop or self.battery_percentage < 10 or self.temperature > 70:
            self.health_status = "CRITICAL"
        elif self.battery_percentage < 20 or self.temperature > 60:
            self.health_status = "ERROR"
        elif self.battery_percentage < 40 or self.obstacle_detected:
            self.health_status = "WARNING"
        else:
            self.health_status = "HEALTHY"

    def publish_status(self):
        """Publish complete robot status"""
        # Update all states
        self.move_towards_waypoint()
        self.update_battery()
        self.update_safety()
        self.update_health()
        
        # Create message
        msg = RobotStatus()
        
        # Header
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        
        # Robot ID
        msg.robot_id = self.robot_id
        msg.robot_name = self.robot_name
        
        # Position
        msg.position_x = float(self.position_x)
        msg.position_y = float(self.position_y)
        msg.orientation_z = float(self.orientation_z)
        
        # Velocity
        msg.linear_velocity = float(self.linear_velocity)
        msg.angular_velocity = float(self.angular_velocity)
        
        # Battery
        msg.battery_voltage = float(self.battery_voltage)
        msg.battery_percentage = int(self.battery_percentage)
        msg.is_charging = self.is_charging
        msg.battery_time_remaining = int(self.battery_time_remaining)
        
        # Safety
        msg.emergency_stop = self.emergency_stop
        msg.obstacle_detected = self.obstacle_detected
        msg.obstacle_distance = float(self.obstacle_distance)
        
        # Mission
        msg.current_mission = self.current_mission
        msg.mission_progress = int(self.mission_progress)
        msg.mission_status = self.mission_status
        
        # Health
        msg.cpu_usage = float(self.cpu_usage)
        msg.temperature = float(self.temperature)
        msg.motors_enabled = self.motors_enabled
        msg.health_status = self.health_status
        
        self.publisher_.publish(msg)
        
        # Log status with icons
        battery_icon = "🔌" if self.is_charging else "🔋"
        motion_icon = "🚀" if self.linear_velocity > 0 else "🛑"
        health_icon = "✅" if self.health_status == "HEALTHY" else "⚠️" if self.health_status == "WARNING" else "❌"
        
        self.get_logger().info(
            f'{health_icon} {self.robot_name} | '
            f'Pos: ({self.position_x:.1f}, {self.position_y:.1f}) | '
            f'{motion_icon} {self.linear_velocity:.2f} m/s | '
            f'{battery_icon} {self.battery_percentage}% | '
            f'Mission: {self.current_mission} ({self.mission_progress}%) | '
            f'Temp: {self.temperature:.1f}°C'
        )


def main(args=None):
    rclpy.init(args=args)
    node = RobotStatusPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(f'\n🤖 {node.robot_name} shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
