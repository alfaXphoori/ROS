#!/usr/bin/env python3

"""
IMU-Based Orientation Controller
=================================

Script demonstrating Inertial Measurement Unit (IMU) usage for:
- Reading accurate orientation (Roll, Pitch, Yaw)
- Precise angle turning using gyroscope
- Compensating for wheel slippage
- Publishing IMU data to ROS2

Features:
- Real-time Dashboard UI (Clear screen update)
- InertialUnit for orientation (quaternion → Euler)
- Gyro for angular velocity
- Keyboard control for precise turns (A = Left, D = Right)

Author: AI Assistant
Level: 3.1 - Inertial Sensors
"""

import rclpy
from rclpy.node import Node
from controller import Robot
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
import math
import sys

class IMUController(Node):
    def __init__(self):
        super().__init__('imu_controller')
        
        # === Configuration Parameters ===
        self.MAX_SPEED = 0.6               # Maximum wheel speed (rad/s)
        self.TURN_SPEED = 0.4              # Rotation speed for turns (rad/s)
        self.ANGLE_TOLERANCE = 1.0         # Angle precision tolerance (degrees)
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
        
        # --- IMU Sensor Setup ---
        # InertialUnit: Provides orientation (roll, pitch, yaw)
        self.imu = self.robot.getDevice('imu')
        self.imu.enable(self.timestep)
        
        # Gyro: Provides angular velocity (rad/s)
        self.gyro = self.robot.getDevice('gyro')
        self.gyro.enable(self.timestep)
        
        # --- ROS2 Publishers ---
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        self.orientation_pub = self.create_publisher(Vector3, '/imu/orientation_euler', 10)
        
        # --- State Variables ---
        self.current_yaw = 0.0
        self.target_yaw = None
        self.mode = "MANUAL"  # MANUAL mode for keyboard control
        self.iteration = 0
        self.status_message = "Waiting for keyboard input..."
        self.turn_increment = 90.0  # Degrees to turn per key press
        
        # Start execution
        self.run()

    def normalize_angle(self, angle):
        """Normalize angle to [-180, 180] degrees"""
        while angle > 180:
            angle -= 360
        while angle < -180:
            angle += 360
        return angle

    def read_imu_data(self):
        """Read all IMU sensor data"""
        # Get orientation
        rpy = self.imu.getRollPitchYaw()
        roll = math.degrees(rpy[0])
        pitch = math.degrees(rpy[1])
        yaw = math.degrees(rpy[2])
        
        # Get angular velocity
        gyro_values = self.gyro.getValues()
        angular_velocity = {'x': gyro_values[0], 'y': gyro_values[1], 'z': gyro_values[2]}
        
        # No accelerometer in Level 3.1 - use gravity constant
        linear_acceleration = {'x': 0.0, 'y': 0.0, 'z': 9.81}
        
        return {
            'orientation': {'roll': roll, 'pitch': pitch, 'yaw': yaw},
            'angular_velocity': angular_velocity,
            'linear_acceleration': linear_acceleration
        }

    def publish_imu_data(self, imu_data):
        """Publish IMU data to ROS2 topics"""
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imu_link'
        
        # Convert Euler to quaternion for message
        roll = math.radians(imu_data['orientation']['roll'])
        pitch = math.radians(imu_data['orientation']['pitch'])
        yaw = math.radians(imu_data['orientation']['yaw'])
        
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        imu_msg.orientation.w = cr * cp * cy + sr * sp * sy
        imu_msg.orientation.x = sr * cp * cy - cr * sp * sy
        imu_msg.orientation.y = cr * sp * cy + sr * cp * sy
        imu_msg.orientation.z = cr * cp * sy - sr * sp * cy
        
        imu_msg.angular_velocity.x = imu_data['angular_velocity']['x']
        imu_msg.angular_velocity.y = imu_data['angular_velocity']['y']
        imu_msg.angular_velocity.z = imu_data['angular_velocity']['z']
        
        imu_msg.linear_acceleration.x = imu_data['linear_acceleration']['x']
        imu_msg.linear_acceleration.y = imu_data['linear_acceleration']['y']
        imu_msg.linear_acceleration.z = imu_data['linear_acceleration']['z']
        
        self.imu_pub.publish(imu_msg)
        
        # Publish Euler angles separately
        euler_msg = Vector3()
        euler_msg.x = imu_data['orientation']['roll']
        euler_msg.y = imu_data['orientation']['pitch']
        euler_msg.z = imu_data['orientation']['yaw']
        self.orientation_pub.publish(euler_msg)

    def turn_to_angle(self, target_angle):
        """Turn robot to specific angle using gyro feedback"""
        error = self.normalize_angle(target_angle - self.current_yaw)
        
        if abs(error) < self.ANGLE_TOLERANCE:
            return True
        
        if error > 0:
            left_speed = -self.TURN_SPEED
            right_speed = self.TURN_SPEED
        else:
            left_speed = self.TURN_SPEED
            right_speed = -self.TURN_SPEED
        
        self.left_motor.setVelocity(left_speed)
        self.right_motor.setVelocity(right_speed)
        return False

    def _print_dashboard(self, imu_data):
        """Print real-time dashboard using ANSI escape codes"""
        # Clear screen and move cursor to home position
        sys.stdout.write("\033[H\033[J")
        
        roll = imu_data['orientation']['roll']
        pitch = imu_data['orientation']['pitch']
        yaw = imu_data['orientation']['yaw']
        gyro_z = imu_data['angular_velocity']['z']
        
        # Visual Compass
        def get_compass(angle):
            angle = self.normalize_angle(angle)
            dirs = ["E ➡️", "NE ↗️", "N ⬆️", "NW ↖️", "W ⬅️", "SW ↙️", "S ⬇️", "SE ↘️"]
            idx = int((angle + 22.5 + 360) % 360 / 45)
            return dirs[idx % 8]

        # Dashboard UI
        print(f"{'='*50}")
        print(f"   🤖  IMU SENSOR DASHBOARD (Real-time)")
        print(f"{'='*50}\n")
        
        print(f"📐 ORIENTATION (Euler)")
        print(f"   Roll:  {roll:7.2f}°")
        print(f"   Pitch: {pitch:7.2f}°")
        print(f"   Yaw:   {yaw:7.2f}°   {get_compass(yaw)}\n")
        
        print(f"💫 ANGULAR VELOCITY (Gyro)")
        print(f"   Z-Axis: {gyro_z:7.3f} rad/s  {'🔄' if abs(gyro_z)>0.1 else '⏸️'}\n")
        
        print(f"{'-'*50}")
        print(f"🎮 STATUS: {self.status_message}")
        
        if self.target_yaw is not None:
             error = self.normalize_angle(self.target_yaw - yaw)
             print(f"🎯 TARGET: {self.target_yaw:>6.1f}°  |  ⚠️ ERROR: {error:>6.1f}°")
        
        print(f"{'-'*50}")
        print(f"⌨️  CONTROLS: A = Turn Left  |  D = Turn Right")
        print(f"{'='*50}")
        sys.stdout.flush()

    def run(self):
        """Main control loop with keyboard control"""
        self.status_message = "Press LEFT or RIGHT arrow to turn"
        
        while self.robot.step(self.timestep) != -1:
            # 1. Read Data
            imu_data = self.read_imu_data()
            self.current_yaw = imu_data['orientation']['yaw']
            
            # 2. Process Keyboard Input
            key = self.keyboard.getKey()
            if key == ord('Q') or key == ord('q'):
                break
            
            # 3. Update Dashboard (Real-time display)
            self._print_dashboard(imu_data)
            
            # 4. Publish to ROS2
            if self.iteration % (1000 // self.timestep // self.PUBLISH_RATE) == 0:
                self.publish_imu_data(imu_data)
            
            # 5. State Machine
            if self.mode == "MANUAL":
                # Check for new turn command
                if key == ord('A') or key == ord('a'):  # A key = Left
                    self.target_yaw = self.normalize_angle(self.current_yaw + self.turn_increment)
                    self.mode = "TURNING"
                    self.status_message = f"⬅️  Turning LEFT to {self.target_yaw:.1f}°"
                elif key == ord('D') or key == ord('d'):  # D key = Right
                    self.target_yaw = self.normalize_angle(self.current_yaw - self.turn_increment)
                    self.mode = "TURNING"
                    self.status_message = f"➡️  Turning RIGHT to {self.target_yaw:.1f}°"
                else:
                    # Keep motors stopped in manual mode
                    self.left_motor.setVelocity(0.0)
                    self.right_motor.setVelocity(0.0)
            
            elif self.mode == "TURNING":
                turn_complete = self.turn_to_angle(self.target_yaw)
                if turn_complete:
                    self.left_motor.setVelocity(0.0)
                    self.right_motor.setVelocity(0.0)
                    self.status_message = f"✅ Reached {self.target_yaw:.1f}°. Ready for next command."
                    self.mode = "MANUAL"
                    self.target_yaw = None
            
            self.iteration += 1
            rclpy.spin_once(self, timeout_sec=0)

def main(args=None):
    rclpy.init(args=args)
    controller = IMUController()
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()