#!/usr/bin/env python3

"""
Accelerometer-Based Motion Controller
======================================

Script demonstrating Accelerometer usage for:
- Detecting linear acceleration in X, Y, Z axes
- Monitoring robot acceleration during movement
- Detecting impacts and collisions
- Publishing acceleration data to ROS2

Features:
- Real-time Dashboard UI with acceleration display
- Accelerometer for linear acceleration (m/s²)
- Collision/impact detection
- Keyboard control (W=Forward, S=Backward, A/D=Turn)
- Beautiful visual acceleration bars

Level: 3.2 - Accelerometer Sensors
"""

import rclpy
from rclpy.node import Node
from controller import Robot
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
import math
import sys

class AccelerometerController(Node):
    def __init__(self):
        super().__init__('accelerometer_controller')
        
        # === Configuration Parameters ===
        self.MAX_SPEED = 10               # Maximum wheel speed (rad/s)
        self.IMPACT_THRESHOLD = 20.0       # Impact detection threshold (m/s²)
        self.PUBLISH_RATE = 10             # ROS2 publish rate (Hz)
        
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
        
        # --- Keyboard Setup ---
        self.keyboard = self.robot.getKeyboard()
        self.keyboard.enable(self.timestep)
        
        # --- Accelerometer Sensor Setup ---
        self.accelerometer = self.robot.getDevice('accelerometer')
        # ตรวจสอบว่ามีอุปกรณ์อยู่จริงหรือไม่
        if self.accelerometer:
            self.accelerometer.enable(self.timestep)
            self.has_accelerometer = True
        else:
            self.has_accelerometer = False
            self.get_logger().error('❌ CRITICAL: Accelerometer device not found!')
            self.get_logger().warn('Please add "Accelerometer { name "accelerometer" }" to your robot in .wbt file')

        
        # --- ROS2 Publishers ---
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        self.accel_pub = self.create_publisher(Vector3, '/accelerometer/data', 10)
        
        # --- State Variables ---
        self.mode = "MANUAL"
        self.iteration = 0
        self.status_message = "Waiting for keyboard input..."
        self.impact_detected = False
        
        # Acceleration history for averaging
        self.accel_history = {'x': [], 'y': [], 'z': []}
        self.HISTORY_SIZE = 5
        
        # Start execution
        self.run()

    def read_accelerometer_data(self):
        """Read and process accelerometer data"""
        if not self.has_accelerometer:
             # ถ้าไม่มี Sensor ให้คืนค่า Default (Simulated Gravity)
             return {
                'raw': {'x': 0.0, 'y': 0.0, 'z': 9.81},
                'smooth': {'x': 0.0, 'y': 0.0, 'z': 9.81},
                'magnitude': 9.81
            }

        accel_values = self.accelerometer.getValues()
        
        # Raw acceleration (m/s²)
        accel_raw = {
            'x': accel_values[0],
            'y': accel_values[1],
            'z': accel_values[2]
        }
        
        # Add to history for smoothing
        for axis in ['x', 'y', 'z']:
            self.accel_history[axis].append(accel_raw[axis])
            if len(self.accel_history[axis]) > self.HISTORY_SIZE:
                self.accel_history[axis].pop(0)
        
        # Smoothed acceleration (running average)
        accel_smooth = {
            'x': sum(self.accel_history['x']) / len(self.accel_history['x']),
            'y': sum(self.accel_history['y']) / len(self.accel_history['y']),
            'z': sum(self.accel_history['z']) / len(self.accel_history['z'])
        }
        
        # Calculate magnitude
        magnitude = math.sqrt(accel_smooth['x']**2 + accel_smooth['y']**2 + accel_smooth['z']**2)
        
        # Detect impacts (sudden high acceleration)
        if magnitude > self.IMPACT_THRESHOLD:
            self.impact_detected = True
        
        return {
            'raw': accel_raw,
            'smooth': accel_smooth,
            'magnitude': magnitude
        }

    def publish_accelerometer_data(self, accel_data):
        """Publish accelerometer data to ROS2 topics"""
        # Publish as IMU message (partial - only acceleration)
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'accelerometer_link'
        
        imu_msg.linear_acceleration.x = accel_data['smooth']['x']
        imu_msg.linear_acceleration.y = accel_data['smooth']['y']
        imu_msg.linear_acceleration.z = accel_data['smooth']['z']
        
        self.imu_pub.publish(imu_msg)
        
        # Publish as Vector3 for easy monitoring
        accel_msg = Vector3()
        accel_msg.x = accel_data['smooth']['x']
        accel_msg.y = accel_data['smooth']['y']
        accel_msg.z = accel_data['smooth']['z']
        self.accel_pub.publish(accel_msg)

    def _print_dashboard(self, accel_data):
        """Print real-time dashboard"""
        # Clear screen
        sys.stdout.write("\033[H\033[J")
        
        accel_x = accel_data['smooth']['x']
        accel_y = accel_data['smooth']['y']
        accel_z = accel_data['smooth']['z']
        magnitude = accel_data['magnitude']
        
        # Visual bar for acceleration
        def get_accel_bar(value, max_val=30.0):
            """Generate visual bar for acceleration"""
            # Normalize to -20 to +20 range
            normalized = int((value / max_val) * 20)
            normalized = max(-20, min(20, normalized))
            
            if normalized >= 0:
                bar = ">" * normalized
                return f"[{bar:>20}]"
            else:
                bar = "<" * abs(normalized)
                return f"[{bar:<20}]"
        
        # Dashboard UI
        print(f"{'='*60}")
        print(f"   🚀  ACCELEROMETER DASHBOARD (Real-time)")
        print(f"{'='*60}\n")
        
        print(f"📊 LINEAR ACCELERATION (m/s²)")
        print(f"   X-axis (Forward):  {accel_x:7.2f}  {get_accel_bar(accel_x)}")
        print(f"   Y-axis (Left):     {accel_y:7.2f}  {get_accel_bar(accel_y)}")
        print(f"   Z-axis (Up):       {accel_z:7.2f}  {get_accel_bar(accel_z)}")
        print(f"   Magnitude:         {magnitude:7.2f}  {'⚠️ HIGH!' if magnitude > 15 else '✓'}\n")
        
        # Gravity compensation display
        gravity_compensated_z = accel_z - 9.81
        print(f"🌍 GRAVITY COMPENSATION")
        print(f"   Z (no gravity):    {gravity_compensated_z:7.2f} m/s²\n")
        
        # Impact detection
        if self.impact_detected:
            print(f"💥 IMPACT DETECTED! Magnitude: {magnitude:.2f} m/s²\n")
            self.impact_detected = False
        
        print(f"{'-'*60}")
        print(f"🎮 STATUS: {self.status_message}")
        print(f"{'-'*60}")
        print(f"⌨️  CONTROLS: W=Forward | S=Backward | A=Left | D=Right")
        print(f"{'='*60}")
        sys.stdout.flush()

    def run(self):
        """Main control loop with keyboard control"""
        self.status_message = "Press W/A/S/D to move"
        
        while self.robot.step(self.timestep) != -1:
            # 1. Read Accelerometer Data
            accel_data = self.read_accelerometer_data()
            
            # 2. Process Keyboard Input
            key = self.keyboard.getKey()
            if key == ord('Q') or key == ord('q'):
                break
            
            # 3. Update Dashboard
            self._print_dashboard(accel_data)
            
            # 4. Publish to ROS2
            if self.iteration % (1000 // self.timestep // self.PUBLISH_RATE) == 0:
                self.publish_accelerometer_data(accel_data)
            
            # 5. Keyboard Control
            left_speed = 0.0
            right_speed = 0.0
            
            if key == ord('W') or key == ord('w'):  # Forward
                left_speed = self.MAX_SPEED
                right_speed = self.MAX_SPEED
                self.status_message = "⬆️  Moving Forward"
            elif key == ord('S') or key == ord('s'):  # Backward
                left_speed = -self.MAX_SPEED
                right_speed = -self.MAX_SPEED
                self.status_message = "⬇️  Moving Backward"
            elif key == ord('A') or key == ord('a'):  # Turn Left
                left_speed = -self.MAX_SPEED * 0.5
                right_speed = self.MAX_SPEED * 0.5
                self.status_message = "⬅️  Turning Left"
            elif key == ord('D') or key == ord('d'):  # Turn Right
                left_speed = self.MAX_SPEED * 0.5
                right_speed = -self.MAX_SPEED * 0.5
                self.status_message = "➡️  Turning Right"
            else:
                self.status_message = "🛑 Stopped. Press W/A/S/D to move"
            
            self.left_motor.setVelocity(left_speed)
            self.right_motor.setVelocity(right_speed)
            
            self.iteration += 1
            rclpy.spin_once(self, timeout_sec=0)

def main(args=None):
    rclpy.init(args=args)
    controller = AccelerometerController()
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()