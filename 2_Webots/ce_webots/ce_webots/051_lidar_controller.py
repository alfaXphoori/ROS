#!/usr/bin/env python3
"""
Level 5.1 - LIDAR Mapping & Obstacle Detection

This controller demonstrates LIDAR sensor usage for:
- 360-degree environment scanning
- Obstacle detection and distance measurement
- Real-time map visualization
- Autonomous obstacle avoidance
- Sector-based navigation

LIDAR (Light Detection and Ranging):
- Emits laser pulses in all directions
- Measures distance to obstacles
- Creates 2D/3D point cloud of environment
- Used in autonomous vehicles, robots, drones

Keyboard Controls:
- W: Move forward
- S: Move backward
- A: Turn left
- D: Turn right
- SPACE: Toggle AUTO/MANUAL mode
- ESC: Exit
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud, LaserScan
from geometry_msgs.msg import Point32
from std_msgs.msg import Header
import sys
import time
import math

# ============== CONFIGURATION ==============

# LIDAR Settings
LIDAR_POINTS = 360          # Number of measurement points
LIDAR_FOV = 2 * math.pi     # Field of view (360 degrees)
LIDAR_MAX_RANGE = 2.0       # Maximum detection range (meters)
PUBLISH_RATE = 10           # Hz

# Navigation Settings
BASE_SPEED = 2.0            # Forward speed in AUTO mode (rad/s)
TURN_SPEED = 1.5            # Turning speed (rad/s)
MAX_SPEED = 6.28            # Maximum wheel speed (rad/s)

# Obstacle Avoidance
SAFE_DISTANCE = 0.6         # Minimum safe distance to obstacles (meters)
CRITICAL_DISTANCE = 0.3     # Emergency stop distance (meters)

# Sector Analysis (divide 360° into sectors for decision making)
FRONT_SECTOR = 30           # Front sector: ±30 degrees
FRONT_LEFT_SECTOR = 60      # Front-left: 30-60 degrees
FRONT_RIGHT_SECTOR = 60     # Front-right: -30 to -60 degrees

# Manual Control
MANUAL_FORWARD_SPEED = 3.0
MANUAL_TURN_SPEED = 2.0

# ===========================================


class LidarController(Node):
    def __init__(self):
        super().__init__('lidar_controller')
        
        # ROS2 Publishers
        self.point_cloud_pub = self.create_publisher(PointCloud, 'lidar_point_cloud', 10)
        self.laser_scan_pub = self.create_publisher(LaserScan, 'lidar_scan', 10)
        
        # Webots Interface
        try:
            from controller import Robot, Keyboard
            self.robot = Robot()
            self.timestep = int(self.robot.getBasicTimeStep())
            
            # Initialize devices
            self.lidar = self.robot.getDevice('lidar')
            self.lidar.enable(self.timestep)
            self.lidar.enablePointCloud()
            
            self.left_motor = self.robot.getDevice('left_motor')
            self.right_motor = self.robot.getDevice('right_motor')
            self.left_motor.setPosition(float('inf'))
            self.right_motor.setPosition(float('inf'))
            self.left_motor.setVelocity(0.0)
            self.right_motor.setVelocity(0.0)
            
            self.imu = self.robot.getDevice('imu')
            self.imu.enable(self.timestep)
            
            self.keyboard = Keyboard()
            self.keyboard.enable(self.timestep)
            
        except Exception as e:
            self.get_logger().error(f'Failed to initialize Webots devices: {e}')
            raise
        
        # Control state
        self.auto_mode = True
        self.manual_left_speed = 0.0
        self.manual_right_speed = 0.0
        
        # LIDAR data storage
        self.range_data = []
        self.sector_distances = {
            'front': float('inf'),
            'front_left': float('inf'),
            'front_right': float('inf'),
            'left': float('inf'),
            'right': float('inf'),
            'back': float('inf')
        }
        
        self.get_logger().info('LIDAR Controller initialized - Level 5.1')
        self.get_logger().info('SPACE: Toggle AUTO/MANUAL | WASD: Manual control | ESC: Exit')
    
    def process_lidar_data(self):
        """Process LIDAR range measurements and analyze environment"""
        range_image = self.lidar.getRangeImage()
        
        if not range_image:
            return
        
        self.range_data = range_image
        num_points = len(range_image)
        
        # Analyze different sectors
        # LIDAR points are arranged counter-clockwise from right side (angle 0)
        # Point 0 = Right (0°), Point 90 = Front (90°), Point 180 = Left (180°), Point 270 = Back (270°)
        
        # Initialize sectors with max range
        for key in self.sector_distances:
            self.sector_distances[key] = LIDAR_MAX_RANGE
        
        # Front sector (85° to 95°, around index 85-95)
        front_indices = list(range((num_points // 4) - 5, (num_points // 4) + 5))
        self.sector_distances['front'] = self._get_sector_min(range_image, front_indices)
        
        # Front-left sector (60° to 120°)
        front_left_indices = list(range((num_points * 1) // 6, (num_points * 2) // 6))
        self.sector_distances['front_left'] = self._get_sector_min(range_image, front_left_indices)
        
        # Front-right sector (-60° to -0°, or 300° to 360°)
        front_right_indices = list(range((num_points * 5) // 6, num_points))
        self.sector_distances['front_right'] = self._get_sector_min(range_image, front_right_indices)
        
        # Left sector (135° to 225°)
        left_indices = list(range((num_points * 3) // 8, (num_points * 5) // 8))
        self.sector_distances['left'] = self._get_sector_min(range_image, left_indices)
        
        # Right sector (-45° to 45°, or 315° to 45°)
        right_indices = list(range(0, num_points // 8)) + list(range((num_points * 7) // 8, num_points))
        self.sector_distances['right'] = self._get_sector_min(range_image, right_indices)
        
        # Back sector (225° to 315°)
        back_indices = list(range((num_points * 5) // 8, (num_points * 7) // 8))
        self.sector_distances['back'] = self._get_sector_min(range_image, back_indices)
    
    def _get_sector_min(self, range_data, indices):
        """Get minimum distance in a sector (ignoring infinity/nan)"""
        valid_ranges = []
        for idx in indices:
            if 0 <= idx < len(range_data):
                r = range_data[idx]
                if r > 0 and r < LIDAR_MAX_RANGE and not math.isinf(r) and not math.isnan(r):
                    valid_ranges.append(r)
        
        return min(valid_ranges) if valid_ranges else LIDAR_MAX_RANGE
    
    def calculate_motor_speeds(self):
        """Autonomous obstacle avoidance using sector analysis"""
        front_dist = self.sector_distances['front']
        front_left_dist = self.sector_distances['front_left']
        front_right_dist = self.sector_distances['front_right']
        left_dist = self.sector_distances['left']
        right_dist = self.sector_distances['right']
        
        # Emergency stop if too close
        if front_dist < CRITICAL_DISTANCE:
            # Back up and turn away from nearest side
            if left_dist > right_dist:
                return -BASE_SPEED, BASE_SPEED  # Back up turning left
            else:
                return BASE_SPEED, -BASE_SPEED  # Back up turning right
        
        # Obstacle ahead - decide which way to turn
        if front_dist < SAFE_DISTANCE:
            # Turn toward the more open side
            if front_left_dist > front_right_dist:
                # Turn left
                return -TURN_SPEED, TURN_SPEED
            else:
                # Turn right
                return TURN_SPEED, -TURN_SPEED
        
        # Path is clear - go forward, but adjust if walls on sides
        left_speed = BASE_SPEED
        right_speed = BASE_SPEED
        
        # Slight adjustment to stay centered in corridor
        if left_dist < SAFE_DISTANCE and right_dist > SAFE_DISTANCE:
            # Too close to left wall, veer right
            left_speed = BASE_SPEED * 1.2
            right_speed = BASE_SPEED * 0.8
        elif right_dist < SAFE_DISTANCE and left_dist > SAFE_DISTANCE:
            # Too close to right wall, veer left
            left_speed = BASE_SPEED * 0.8
            right_speed = BASE_SPEED * 1.2
        
        return left_speed, right_speed
    
    def handle_keyboard(self):
        """Process keyboard input for manual control"""
        key = self.keyboard.getKey()
        
        if key == -1:
            return
        
        # Toggle mode
        if key == ord(' '):  # SPACE
            self.auto_mode = not self.auto_mode
            if not self.auto_mode:
                self.manual_left_speed = 0.0
                self.manual_right_speed = 0.0
        
        # Exit
        elif key == 27:  # ESC
            self.get_logger().info('ESC pressed - shutting down')
            rclpy.shutdown()
            sys.exit(0)
        
        # Manual controls (only in manual mode)
        if not self.auto_mode:
            if key == ord('W'):
                self.manual_left_speed = MANUAL_FORWARD_SPEED
                self.manual_right_speed = MANUAL_FORWARD_SPEED
            elif key == ord('S'):
                self.manual_left_speed = -MANUAL_FORWARD_SPEED
                self.manual_right_speed = -MANUAL_FORWARD_SPEED
            elif key == ord('A'):
                self.manual_left_speed = -MANUAL_TURN_SPEED
                self.manual_right_speed = MANUAL_TURN_SPEED
            elif key == ord('D'):
                self.manual_left_speed = MANUAL_TURN_SPEED
                self.manual_right_speed = -MANUAL_TURN_SPEED
            else:
                # No key pressed - stop
                self.manual_left_speed = 0.0
                self.manual_right_speed = 0.0
    
    def set_motor_speeds(self, left, right):
        """Set motor speeds with safety limits"""
        left = max(-MAX_SPEED, min(MAX_SPEED, left))
        right = max(-MAX_SPEED, min(MAX_SPEED, right))
        self.left_motor.setVelocity(left)
        self.right_motor.setVelocity(right)
    
    def publish_ros_data(self):
        """Publish LIDAR data as ROS2 messages"""
        if not self.range_data:
            return
        
        # Publish LaserScan (standard ROS message)
        scan_msg = LaserScan()
        scan_msg.header = Header()
        scan_msg.header.stamp = self.get_clock().now().to_msg()
        scan_msg.header.frame_id = 'lidar_frame'
        
        scan_msg.angle_min = 0.0
        scan_msg.angle_max = LIDAR_FOV
        scan_msg.angle_increment = LIDAR_FOV / len(self.range_data)
        scan_msg.range_min = 0.1
        scan_msg.range_max = LIDAR_MAX_RANGE
        scan_msg.ranges = [float(r) for r in self.range_data]
        
        self.laser_scan_pub.publish(scan_msg)
        
        # Publish PointCloud for visualization
        cloud_msg = PointCloud()
        cloud_msg.header = scan_msg.header
        
        for i, distance in enumerate(self.range_data):
            if distance > 0 and distance < LIDAR_MAX_RANGE:
                angle = i * LIDAR_FOV / len(self.range_data)
                x = distance * math.cos(angle)
                y = distance * math.sin(angle)
                cloud_msg.points.append(Point32(x=float(x), y=float(y), z=0.0))
        
        self.point_cloud_pub.publish(cloud_msg)
    
    def display_dashboard(self):
        """Display LIDAR data and navigation info on terminal"""
        # Clear screen and move cursor to top
        print('\033[H\033[J', end='')
        
        print("=" * 60)
        print("  LEVEL 5.1 - LIDAR MAPPING & OBSTACLE DETECTION")
        print("=" * 60)
        
        # Mode indicator
        mode_text = "🤖 AUTO MODE" if self.auto_mode else "🎮 MANUAL MODE"
        print(f"\nMode: {mode_text}")
        
        # Sector distances (display as a radar-like visualization)
        print("\n┌───  LIDAR SECTOR ANALYSIS ───┐")
        print(f"│                              │")
        print(f"│      ↖ FL  ↑ F   FR ↗        │")
        print(f"│      {self.sector_distances['front_left']:4.2f}  {self.sector_distances['front']:4.2f}  {self.sector_distances['front_right']:4.2f}        │")
        print(f"│                              │")
        print(f"│   L ← {self.sector_distances['left']:4.2f}    🤖   {self.sector_distances['right']:4.2f} → R  │")
        print(f"│                              │")
        print(f"│              {self.sector_distances['back']:4.2f}            │")
        print(f"│              ↓ B             │")
        print("└──────────────────────────────┘")
        
        # Obstacle warning
        closest_dist = min(self.sector_distances.values())
        if closest_dist < CRITICAL_DISTANCE:
            print(f"\n⚠️  CRITICAL: Obstacle at {closest_dist:.2f}m!")
        elif closest_dist < SAFE_DISTANCE:
            print(f"\n⚡ WARNING: Obstacle at {closest_dist:.2f}m")
        else:
            print(f"\n✅ Clear path - Closest obstacle: {closest_dist:.2f}m")
        
        # Motor speeds
        if self.auto_mode:
            left_speed, right_speed = self.calculate_motor_speeds()
            print(f"\nMotor Speeds: L={left_speed:+.2f}  R={right_speed:+.2f}")
        else:
            print(f"\nMotor Speeds: L={self.manual_left_speed:+.2f}  R={self.manual_right_speed:+.2f}")
        
        # Visual range indicator (horizontal bar for front distance)
        bar_length = 40
        if self.sector_distances['front'] < LIDAR_MAX_RANGE:
            filled = int((self.sector_distances['front'] / LIDAR_MAX_RANGE) * bar_length)
            bar = '█' * filled + '░' * (bar_length - filled)
        else:
            bar = '░' * bar_length
        print(f"\nFront Distance: [{bar}] {self.sector_distances['front']:.2f}m")
        
        # IMU orientation
        roll, pitch, yaw = self.imu.getRollPitchYaw()
        print(f"\nOrientation: Yaw={math.degrees(yaw):6.1f}°")
        
        # Instructions
        print("\n" + "─" * 60)
        print("SPACE: Toggle Mode | WASD: Manual Control | ESC: Exit")
        print("=" * 60)
    
    def run(self):
        """Main control loop"""
        while self.robot.step(self.timestep) != -1 and rclpy.ok():
            # Process sensors
            self.process_lidar_data()
            self.handle_keyboard()
            
            # Control motors
            if self.auto_mode:
                left_speed, right_speed = self.calculate_motor_speeds()
            else:
                left_speed = self.manual_left_speed
                right_speed = self.manual_right_speed
            
            self.set_motor_speeds(left_speed, right_speed)
            
            # Publish ROS data
            self.publish_ros_data()
            
            # Update display
            self.display_dashboard()
            
            # ROS2 spin
            rclpy.spin_once(self, timeout_sec=0)
            
            time.sleep(1.0 / PUBLISH_RATE)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        controller = LidarController()
        controller.run()
    except KeyboardInterrupt:
        print('\nShutdown requested')
    except Exception as e:
        print(f'Error: {e}')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
