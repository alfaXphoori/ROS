#!/usr/bin/env python3

"""
Autonomous Walk & Avoid
=======================

Script to make the robot autonomously walk forward and avoid obstacles
using data from 5 distance sensors.

Logic:
1. Normal: Move forward at full speed
2. Obstacle detected in front (< 0.5m): Turn away
3. Obstacle detected on sides: Adjust slightly away

"""

import rclpy
from rclpy.node import Node
from controller import Robot

class AutonomousWalker(Node):
    def __init__(self):
        super().__init__('autonomous_walker')
        
        # === Configuration Parameters ===
        self.MAX_SPEED = 0.8           # Maximum wheel speed (rad/s)
        self.AVOID_DIST = 0.35          # Obstacle avoidance distance threshold (meters)
        self.SIDE_ADJUST_DIST = 0.2    # Side wall adjustment distance (meters)
        self.TURN_SPEED_RATIO = 0.8    # Speed ratio when turning (0.0 - 1.0)
        
        # Initialize Webots robot
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())
        
        # --- Motor setup ---
        self.left_motor = self.robot.getDevice('left_motor')
        self.right_motor = self.robot.getDevice('right_motor')
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)
        
        # --- Sensor setup ---
        self.sensors = {}
        sensor_names = ['ds_front', 'ds_front_left', 'ds_front_right', 'ds_left', 'ds_right']
        
        for name in sensor_names:
            self.sensors[name] = self.robot.getDevice(name)
            self.sensors[name].enable(self.timestep)
        
        # Start execution
        self.run()

    def get_distance(self, sensor_name):
        """Convert raw value (0-1000) to meters"""
        value = self.sensors[sensor_name].getValue()
        # According to lookup table: 1000 = 1.0m, 0 = 0m
        # Therefore: low value = close, high value = far
        return value / 1000.0

    def _print_sensor_status(self, front, left, right, front_left, front_right):
        """Print beautiful formatted sensor readings"""
        def get_bar(distance, max_dist=1.0):
            """Generate a visual bar representing distance"""
            bar_length = int((distance / max_dist) * 20)
            bar_length = min(20, max(0, bar_length))
            
            if distance < 0.2:
                color = "🔴"  # Danger - very close
                bar = "█" * bar_length
            elif distance < 0.5:
                color = "🟡"  # Warning - close
                bar = "▓" * bar_length
            else:
                color = "🟢"  # Safe - far
                bar = "░" * bar_length
            
            return f"{color} {bar:<20} {distance:.2f}m"
        
        print("\r" + "─"*80, end="")
        print(f"\r┌─ 📡 SENSOR READINGS {'─'*56}")
        print(f"│  Front Left:  {get_bar(front_left)}")
        print(f"│  Front:       {get_bar(front)}")
        print(f"│  Front Right: {get_bar(front_right)}")
        print(f"│  Left:        {get_bar(left)}")
        print(f"│  Right:       {get_bar(right)}")
        print(f"└{'─'*78}")

    def run(self):
        self.get_logger().info('🚀 STARTING AUTONOMOUS WALK')
        print("="*80)
        print("🤖 AUTONOMOUS WALKER - Configuration")
        print("="*80)
        print(f"  Max Speed:           {self.MAX_SPEED} rad/s")
        print(f"  Avoid Distance:      {self.AVOID_DIST} m")
        print(f"  Side Adjust Dist:    {self.SIDE_ADJUST_DIST} m")
        print(f"  Turn Speed Ratio:    {self.TURN_SPEED_RATIO}")
        print("="*80)
        print()
        
        while self.robot.step(self.timestep) != -1:
            # 1. Read all sensor values (in meters)
            d_front = self.get_distance('ds_front')
            d_front_left = self.get_distance('ds_front_left')
            d_front_right = self.get_distance('ds_front_right')
            d_left = self.get_distance('ds_left')
            d_right = self.get_distance('ds_right')

            # --- Display sensor data continuously ---
            self._print_sensor_status(d_front, d_left, d_right, d_front_left, d_front_right)

            # 2. Decision making logic
            left_speed = self.MAX_SPEED
            right_speed = self.MAX_SPEED 

            # Case 1: Obstacle detected straight ahead or diagonal front
            status = "🟢 FORWARD"
            if d_front < self.AVOID_DIST or d_front_left < self.AVOID_DIST or d_front_right < self.AVOID_DIST:
                # Obstacle detected! Must turn
                
                # Decide whether to turn left or right (go to the more open side)
                if d_left > d_right:
                    # Left side is more open -> Turn left (left wheel reverse, right wheel forward)
                    left_speed = -self.MAX_SPEED * self.TURN_SPEED_RATIO
                    right_speed = self.MAX_SPEED * self.TURN_SPEED_RATIO
                    status = "🔴 ⬅️  TURN LEFT"
                else:
                    # Right side is more open -> Turn right
                    left_speed = self.MAX_SPEED * self.TURN_SPEED_RATIO
                    right_speed = -self.MAX_SPEED * self.TURN_SPEED_RATIO
                    status = "🔴 ➡️  TURN RIGHT"
            
            # Case 2: Path is clear, continue forward
            else:
                # Additional feature: If side is too close, adjust slightly away (Wall Following behavior)
                if d_left < self.SIDE_ADJUST_DIST:
                    left_speed += 0.1  # Accelerate left wheel to veer right
                    right_speed -= 0.1
                    status = "🟡 ADJUST RIGHT"
                elif d_right < self.SIDE_ADJUST_DIST:
                    left_speed -= 0.1
                    right_speed += 0.1 # Accelerate right wheel to veer left
                    status = "🟡 ADJUST LEFT"
            
            # Display current action
            print(f"  ➤ Action: {status}")
            print()

            # 3. Command motors
            self.left_motor.setVelocity(left_speed)
            self.right_motor.setVelocity(right_speed)
            
            # Let ROS process callbacks
            rclpy.spin_once(self, timeout_sec=0)

def main(args=None):
    rclpy.init(args=args)
    controller = AutonomousWalker()
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()