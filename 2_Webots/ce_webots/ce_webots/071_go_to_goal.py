#!/usr/bin/env python3

"""
Level 7.1: Go to Goal (Navigation + Obstacle Avoidance + Stuck Recovery)
=========================================================================
Navigate to waypoints with advanced obstacle avoidance and stuck recovery

Features:
1. Odometry: Calculate position (X, Y, Theta) from encoders and IMU
2. Go-to-Goal: Navigate to specified waypoints using angle/distance control
3. Obstacle Avoidance: 052 SLAM algorithm (Emergency/Critical/Mild/Wall/Clear)
4. Stuck Detection: Automatically detect and recover from stuck situations
5. Waypoint Loop: Visit multiple points in sequence

Author: AI Assistant
"""

import rclpy
from rclpy.node import Node
from controller import Robot
import math
import sys

class GoToGoal(Node):
    def __init__(self):
        super().__init__('go_to_goal')
        
        # Initialize Webots
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())
        
        # --- Robot Physical Parameters (Fixed to match World file) ---
        self.WHEEL_RADIUS = 0.08   # meters
        self.AXLE_LENGTH = 0.24    # meters
        
        # --- Speed Parameters ---
        self.MAX_SPEED = 4.0           
        self.TURN_SPEED_RATIO = 0.5    
        self.SLOW_SPEED_RATIO = 0.5    
        
        # --- Distance Parameters ---
        self.EMERGENCY_DISTANCE = 0.3 # meters - Too close! Reverse
        self.CRITICAL_DISTANCE = 0.4  # meters - Trigger Spin
        self.SAFE_DISTANCE = 0.7      # meters - Avoidance distance
        self.MAX_LIDAR_RANGE = 4.0
        
        # --- Hardware Setup ---
        self.left_motor = self.robot.getDevice('left_motor')
        self.right_motor = self.robot.getDevice('right_motor')
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)
        
        self.left_encoder = self.robot.getDevice('left_encoder')
        self.right_encoder = self.robot.getDevice('right_encoder')
        self.left_encoder.enable(self.timestep)
        self.right_encoder.enable(self.timestep)
        
        self.imu = self.robot.getDevice('imu')
        self.imu.enable(self.timestep)
        
        self.lidar = self.robot.getDevice('lidar')
        self.lidar.enable(self.timestep)
        
        # --- Navigation State ---
        self.pose = {'x': 0.0, 'y': 0.0, 'theta': 1.5708}  # Start facing North
        self.last_enc_l = 0.0
        self.last_enc_r = 0.0
        
        # Waypoints
        self.goals = [
            {'x': 2.0, 'y': 0.8, 'name': 'Point A (Red)'},
            {'x': 0.0, 'y': -2.2, 'name': 'Point B (Green)'},
            {'x': -2.2, 'y': -0.5, 'name': 'Point C (Blue)'},
            {'x': -1.2, 'y': 1.8, 'name': 'Point D (Yellow)'},
            {'x': 0.0, 'y': 0.0, 'name': 'Home'}
        ]
        self.current_goal_idx = 0
        self.state = "TURN"  # States: TURN, MOVE, REACHED
        self.wait_counter = 0
        
        # Obstacle avoidance state
        self.stuck_timer = 0
        self.stuck_action = "NONE"  # NONE, LEFT, RIGHT, BACK, STOP
        self.stop_timer = 0  # Timer for stop phase before action
        self.target_turn_angle = None  # Target angle for precise turns (radians)
        self.turn_start_angle = None  # Starting angle when turn begins
        
        self.run()

    def update_odometry(self):
        """Calculate current position from Encoders and IMU"""
        curr_l = self.left_encoder.getValue()
        curr_r = self.right_encoder.getValue()
        
        dl = (curr_l - self.last_enc_l) * self.WHEEL_RADIUS
        dr = (curr_r - self.last_enc_r) * self.WHEEL_RADIUS
        self.last_enc_l = curr_l
        self.last_enc_r = curr_r
        
        dist = (dl + dr) / 2.0
        
        # Use IMU for accurate heading
        rpy = self.imu.getRollPitchYaw()
        if rpy and not math.isnan(rpy[2]):
            self.pose['theta'] = rpy[2]
            
        self.pose['x'] += dist * math.cos(self.pose['theta'])
        self.pose['y'] += dist * math.sin(self.pose['theta'])

    def normalize_angle(self, angle):
        """Normalize angle to be within -PI to PI"""
        while angle > math.pi: angle -= 2.0 * math.pi
        while angle < -math.pi: angle += 2.0 * math.pi
        return angle
    
    def detect_obstacle(self, ranges):
        """Detect obstacles using 052 algorithm"""
        if not ranges:
            return 10.0, 10.0, 10.0
        
        n = len(ranges)
        idx_front = n // 2
        idx_left = n // 4
        idx_right = 3 * n // 4
        width = n // 12
        
        def get_min_in_sector(center_idx, width):
            start = max(0, center_idx - width)
            end = min(n, center_idx + width)
            sector = ranges[start:end]
            return min([r for r in sector if r != float('inf')], default=10.0)

        min_front = get_min_in_sector(idx_front, width)
        min_left = get_min_in_sector(idx_left, width)
        min_right = get_min_in_sector(idx_right, width)
        
        return min_front, min_left, min_right
    
    def avoid_obstacle(self, ranges):
        """052 SLAM obstacle avoidance algorithm with stop-before-action"""
        if not ranges:
            return None
        
        min_front, min_left, min_right = self.detect_obstacle(ranges)
        
        # Get back sensor reading
        n = len(ranges)
        idx_back = 0  # Back is at index 0 in lidar
        back_width = n // 12
        back_sector = ranges[:back_width] + ranges[-back_width:]
        min_back = min([r for r in back_sector if r != float('inf')], default=10.0)
        
        # Handle stop phase first
        if self.stop_timer > 0:
            self.stop_timer -= 1
            if self.stop_timer == 0:
                # Stop complete, start main action
                if self.stuck_action == "BACK":
                    self.stuck_timer = 15
                elif self.stuck_action in ["LEFT", "RIGHT"]:
                    if self.target_turn_angle is not None:
                        # Precise angle turn - record starting angle
                        self.turn_start_angle = self.pose['theta']
                    else:
                        # Timer-based turn
                        self.stuck_timer = 30
            return 0.0, 0.0, "🛑 STOPPING!"
        
        # Handle precise angle turns (for EMERGENCY)
        if self.target_turn_angle is not None and self.turn_start_angle is not None:
            angle_turned = abs(self.normalize_angle(self.pose['theta'] - self.turn_start_angle))
            
            if angle_turned >= self.target_turn_angle:
                # Turn complete
                self.target_turn_angle = None
                self.turn_start_angle = None
                self.stuck_action = "NONE"
            else:
                # Continue turning
                remaining = math.degrees(self.target_turn_angle - angle_turned)
                if self.stuck_action == "LEFT":
                    return -self.MAX_SPEED * 0.6, self.MAX_SPEED * 0.6, f"🔄 Emergency Turn Left ({remaining:.0f}° left)"
                elif self.stuck_action == "RIGHT":
                    return self.MAX_SPEED * 0.6, -self.MAX_SPEED * 0.6, f"🔄 Emergency Turn Right ({remaining:.0f}° left)"
        
        # Handle timer-based actions (commit to action, don't change mid-execution)
        if self.stuck_timer > 0:
            self.stuck_timer -= 1
            
            if self.stuck_action == "BACK":
                return -self.MAX_SPEED * 0.5, -self.MAX_SPEED * 0.5, "⚠️ REVERSING!"
                    
            elif self.stuck_action == "LEFT":
                return -self.MAX_SPEED * 0.6, self.MAX_SPEED * 0.6, "🔄 Avoiding (Left)"
                    
            elif self.stuck_action == "RIGHT":
                return self.MAX_SPEED * 0.6, -self.MAX_SPEED * 0.6, "🔄 Avoiding (Right)"
        
        # 1. EMERGENCY: Too close - Turn 120 degrees
        if min_front < self.EMERGENCY_DISTANCE:
            # Stop first, then decide action
            self.stop_timer = 5  # Stop for ~5 iterations
            self.target_turn_angle = math.radians(120)  # Turn exactly 120 degrees
            # Check if can reverse
            if min_back > self.EMERGENCY_DISTANCE:
                self.stuck_action = "BACK"
                self.target_turn_angle = None  # No turn for reversing
                return 0.0, 0.0, "🚨 EMERGENCY STOP!"
            else:
                # Can't reverse, turn 120 degrees
                if min_left > min_right:
                    self.stuck_action = "LEFT"
                else:
                    self.stuck_action = "RIGHT"
                return 0.0, 0.0, "🚨 EMERGENCY! Will turn 120°"

        # 2. OBSTACLE AHEAD: Turn towards open space
        if min_front < self.CRITICAL_DISTANCE:
            # Stop first
            self.stop_timer = 5
            if min_left > min_right and min_left > self.CRITICAL_DISTANCE:
                self.stuck_action = "LEFT"
            elif min_right > self.CRITICAL_DISTANCE:
                self.stuck_action = "RIGHT"
            else:
                # Both sides blocked, reverse if possible
                if min_back > self.EMERGENCY_DISTANCE:
                    self.stuck_action = "BACK"
                else:
                    # Completely surrounded, turn in place anyway
                    self.stuck_action = "LEFT"
            return 0.0, 0.0, "🚧 STOP! Obstacle Ahead"

        # 3. MILD AVOIDANCE
        if min_front < self.SAFE_DISTANCE:
            if min_left > min_right:
                return self.MAX_SPEED * 0.3, self.MAX_SPEED * 0.8, "🛑 Blocked"
            else:
                return self.MAX_SPEED * 0.8, self.MAX_SPEED * 0.3, "🛑 Blocked"

        # 4. SIDE WALL FOLLOWING
        if min_left < self.CRITICAL_DISTANCE:
            return self.MAX_SPEED * 0.8, self.MAX_SPEED * 0.6, "⚠️ Wall Left"
        elif min_right < self.CRITICAL_DISTANCE:
            return self.MAX_SPEED * 0.6, self.MAX_SPEED * 0.8, "⚠️ Wall Right"

        return None

    def navigate_to_goal(self, ranges):
        """Navigate to goal with obstacle avoidance"""
        goal = self.goals[self.current_goal_idx]
        
        # Check for obstacles (priority)
        avoid_result = self.avoid_obstacle(ranges)
        if avoid_result is not None and self.state != "REACHED":
            left_speed, right_speed, status = avoid_result
            return left_speed, right_speed, status, goal['name']
        
        # Calculate navigation
        dx = goal['x'] - self.pose['x']
        dy = goal['y'] - self.pose['y']
        distance = math.sqrt(dx**2 + dy**2)
        target_angle = math.atan2(dy, dx)
        angle_diff = self.normalize_angle(target_angle - self.pose['theta'])
        
        left_speed = 0.0
        right_speed = 0.0
        status = "Thinking..."
        
        # State machine
        if self.state == "REACHED":
            status = "✅ ARRIVED! Waiting..."
            self.wait_counter += 1
            if self.wait_counter > 50:
                self.current_goal_idx = (self.current_goal_idx + 1) % len(self.goals)
                self.state = "TURN"
                self.wait_counter = 0

        elif distance < 0.15:
            self.state = "REACHED"
            left_speed = 0.0
            right_speed = 0.0
            
        elif self.state == "TURN":
            if abs(angle_diff) > 0.1:
                status = f"🔄 Turning ({math.degrees(angle_diff):.1f}°)"
                turn_speed = 2.0
                if angle_diff > 0:
                    left_speed = -turn_speed
                    right_speed = turn_speed
                else:
                    left_speed = turn_speed
                    right_speed = -turn_speed
            else:
                self.state = "MOVE"
                
        elif self.state == "MOVE":
            if abs(angle_diff) > 0.5:
                self.state = "TURN"
            else:
                status = f"⬆️  Moving ({distance:.2f}m)"
                forward_speed = min(self.MAX_SPEED, distance * 2.0)
                forward_speed = max(forward_speed, 1.0)
                
                correction = angle_diff * 2.0
                
                left_speed = forward_speed - correction
                right_speed = forward_speed + correction

        return left_speed, right_speed, status, goal['name']

    def print_dashboard(self, status, goal_name, ranges):
        sys.stdout.write("\033[H\033[J")
        
        min_front, min_left, min_right = self.detect_obstacle(ranges)
        
        print("="*50)
        print(f"   🎯 GO TO GOAL + AVOIDANCE (v7.1)")
        print("="*50)
        print(f" Position: X={self.pose['x']:.2f} Y={self.pose['y']:.2f}")
        print(f" Heading:  {math.degrees(self.pose['theta']):.1f}°")
        print("-" * 50)
        
        f_str = f"⚠️ {min_front:.2f}m" if min_front < 0.7 else f"{min_front:.2f}m"
        print(f" LIDAR: Front={f_str} | Left={min_left:.2f}m | Right={min_right:.2f}m")
        print("-" * 50)
        print(f" TARGET:   {goal_name} ({self.goals[self.current_goal_idx]['x']}, {self.goals[self.current_goal_idx]['y']})")
        print(f" STATUS:   {status}")
        print("="*50)
        sys.stdout.flush()

    def run(self):
        while self.robot.step(self.timestep) != -1:
            self.update_odometry()
            
            ranges = self.lidar.getRangeImage()
            l, r, status, g_name = self.navigate_to_goal(ranges)
            
            self.left_motor.setVelocity(l)
            self.right_motor.setVelocity(r)
            
            self.print_dashboard(status, g_name, ranges)

def main(args=None):
    rclpy.init(args=args)
    controller = GoToGoal()
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
