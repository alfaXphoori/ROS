#!/usr/bin/env python3

"""
RGB Camera Controller - Color Detection
========================================

Script demonstrating RGB Camera usage for:
- Capturing color images from robot camera
- Processing raw pixel data (BGRA format)
- Center pixel color detection
- Multi-color object detection
- Publishing camera images to ROS2

Features:
- Real-time color detection at center pixel
- Visual RGB channel bars
- Configurable color thresholds
- Keyboard control for navigation
- ROS2 image publishing

Level: 4.1 - Visual Sensors (Camera)
"""

import rclpy
from rclpy.node import Node
from controller import Robot
from sensor_msgs.msg import Image
import sys

class CameraController(Node):
    def __init__(self):
        super().__init__('camera_controller')
        
        # === Configuration Parameters ===
        self.MAX_SPEED = 6.0               # Maximum wheel speed (rad/s)
        self.TURN_SPEED_RATIO = 0.5        # Turn speed ratio (0-1)
        self.PUBLISH_RATE = 10             # ROS2 image publish rate (Hz)
        
        # Color Detection Thresholds (RGB values 0-255)
        self.RED_MIN = [150, 0, 0]         # Red color minimum [R, G, B]
        self.RED_MAX = [255, 100, 100]     # Red color maximum
        self.GREEN_MIN = [0, 150, 0]       # Green minimum
        self.GREEN_MAX = [100, 255, 100]   # Green maximum
        self.BLUE_MIN = [0, 0, 150]        # Blue minimum
        self.BLUE_MAX = [100, 100, 255]    # Blue maximum
        self.YELLOW_MIN = [150, 150, 0]    # Yellow minimum
        self.YELLOW_MAX = [255, 255, 100]  # Yellow maximum
        self.ORANGE_MIN = [150, 80, 0]     # Orange minimum
        self.ORANGE_MAX = [255, 150, 80]   # Orange maximum
        self.PURPLE_MIN = [100, 0, 150]    # Purple minimum
        self.PURPLE_MAX = [255, 50, 255]   # Purple maximum
        self.DARK_THRESHOLD = 100          # Below this = dark
        self.BRIGHT_THRESHOLD = 200        # Above this = bright/white
        
        # Initialize Webots robot
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())
        
        # --- Motor Setup ---
        self.left_motor = self.robot.getDevice('left_motor')
        self.right_motor = self.robot.getDevice('right_motor')
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)
        
        # --- Keyboard Setup ---
        self.keyboard = self.robot.getKeyboard()
        self.keyboard.enable(self.timestep)
        
        # --- Camera Setup ---
        self.camera = self.robot.getDevice('camera')
        self.camera.enable(self.timestep)
        self.width = self.camera.getWidth()
        self.height = self.camera.getHeight()
        
        self.get_logger().info(f'📷 Camera Initialized: {self.width}x{self.height}')
        
        # --- ROS2 Publishers ---
        self.image_pub = self.create_publisher(Image, '/camera/image_raw', 10)
        
        # --- State Variables ---
        self.iteration = 0
        self.detected_color = "Unknown"
        
        # Start execution
        self.run()

    def detect_color(self, r, g, b):
        """Detect color based on RGB values and thresholds"""
        # Check each color range
        if (self.RED_MIN[0] <= r <= self.RED_MAX[0] and 
            self.RED_MIN[1] <= g <= self.RED_MAX[1] and 
            self.RED_MIN[2] <= b <= self.RED_MAX[2]):
            return "🔴 RED"
        
        elif (self.GREEN_MIN[0] <= r <= self.GREEN_MAX[0] and 
              self.GREEN_MIN[1] <= g <= self.GREEN_MAX[1] and 
              self.GREEN_MIN[2] <= b <= self.GREEN_MAX[2]):
            return "🟢 GREEN"
        
        elif (self.BLUE_MIN[0] <= r <= self.BLUE_MAX[0] and 
              self.BLUE_MIN[1] <= g <= self.BLUE_MAX[1] and 
              self.BLUE_MIN[2] <= b <= self.BLUE_MAX[2]):
            return "🔵 BLUE"
        
        elif (self.YELLOW_MIN[0] <= r <= self.YELLOW_MAX[0] and 
              self.YELLOW_MIN[1] <= g <= self.YELLOW_MAX[1] and 
              self.YELLOW_MIN[2] <= b <= self.YELLOW_MAX[2]):
            return "🟡 YELLOW"
        
        elif (self.ORANGE_MIN[0] <= r <= self.ORANGE_MAX[0] and 
              self.ORANGE_MIN[1] <= g <= self.ORANGE_MAX[1] and 
              self.ORANGE_MIN[2] <= b <= self.ORANGE_MAX[2]):
            return "🟠 ORANGE"
        
        elif (self.PURPLE_MIN[0] <= r <= self.PURPLE_MAX[0] and 
              self.PURPLE_MIN[1] <= g <= self.PURPLE_MAX[1] and 
              self.PURPLE_MIN[2] <= b <= self.PURPLE_MAX[2]):
            return "🟣 PURPLE"
        
        elif r < self.DARK_THRESHOLD and g < self.DARK_THRESHOLD and b < self.DARK_THRESHOLD:
            return "⚫ DARK (Floor/Shadow)"
        
        elif r > self.BRIGHT_THRESHOLD and g > self.BRIGHT_THRESHOLD and b > self.BRIGHT_THRESHOLD:
            return "⚪ WHITE (Background/Wall)"
        
        else:
            # Determine dominant channel
            dominant = max([('R', r), ('G', g), ('B', b)], key=lambda x: x[1])
            return f"🎨 Mixed (Dominant: {dominant[0]}={dominant[1]})"

    def process_camera_image(self):
        """Read and process camera image data"""
        # Get raw image data (BGRA format from Webots)
        image_data = self.camera.getImage()
        
        if image_data:
            # Analyze center pixel
            cx = self.width // 2
            cy = self.height // 2
            
            # Calculate pixel index (4 bytes per pixel: BGRA)
            pixel_index = (cy * self.width + cx) * 4
            
            # Extract color values (BGRA order)
            blue = image_data[pixel_index]
            green = image_data[pixel_index + 1]
            red = image_data[pixel_index + 2]
            # alpha = image_data[pixel_index + 3]  # Not used
            
            # Detect color
            self.detected_color = self.detect_color(red, green, blue)
            
            # Display dashboard
            self._print_dashboard(red, green, blue)
            
            # Publish to ROS2
            if self.iteration % (1000 // self.timestep // self.PUBLISH_RATE) == 0:
                self.publish_ros_image(image_data)

    def publish_ros_image(self, raw_data):
        """Publish camera image to ROS2 topic"""
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_link"
        msg.height = self.height
        msg.width = self.width
        msg.encoding = "bgra8"  # Webots camera returns BGRA format
        msg.is_bigendian = False
        msg.step = 4 * self.width
        msg.data = raw_data
        self.image_pub.publish(msg)

    def _print_dashboard(self, r, g, b):
        """Print real-time dashboard with color detection"""
        sys.stdout.write("\033[H\033[J")  # Clear screen
        
        print(f"{'='*50}")
        print(f"   📷  CAMERA CONTROLLER - Color Detection")
        print(f"{'='*50}\n")
        
        print(f"📸 CAMERA INFO")
        print(f"   Resolution: {self.width} x {self.height}")
        print(f"   Format: BGRA (4 bytes/pixel)\n")
        
        print(f"🎯 CENTER PIXEL COLOR (RGB):")
        print(f"   🔴 Red:   {r:<3} {'█' * min(r//10, 25)}")
        print(f"   🟢 Green: {g:<3} {'█' * min(g//10, 25)}")
        print(f"   🔵 Blue:  {b:<3} {'█' * min(b//10, 25)}\n")
        
        print(f"💡 DETECTION: {self.detected_color}\n")
        
        print(f"{'-'*50}")
        print(f"⌨️  CONTROLS: W=Forward | S=Backward | A=Left | D=Right")
        print(f"{'='*50}")
        sys.stdout.flush()

    def run(self):
        """Main control loop"""
        while self.robot.step(self.timestep) != -1:
            # 1. Process camera image
            self.process_camera_image()
            
            # 2. Handle keyboard input
            key = self.keyboard.getKey()
            left_speed = 0.0
            right_speed = 0.0
            
            if key == ord('W') or key == ord('w'):  # Forward
                left_speed = self.MAX_SPEED
                right_speed = self.MAX_SPEED
            elif key == ord('S') or key == ord('s'):  # Backward
                left_speed = -self.MAX_SPEED
                right_speed = -self.MAX_SPEED
            elif key == ord('A') or key == ord('a'):  # Turn Left
                left_speed = -self.MAX_SPEED * self.TURN_SPEED_RATIO
                right_speed = self.MAX_SPEED * self.TURN_SPEED_RATIO
            elif key == ord('D') or key == ord('d'):  # Turn Right
                left_speed = self.MAX_SPEED * self.TURN_SPEED_RATIO
                right_speed = -self.MAX_SPEED * self.TURN_SPEED_RATIO
            elif key == ord('Q') or key == ord('q'):  # Quit
                break
            
            self.left_motor.setVelocity(left_speed)
            self.right_motor.setVelocity(right_speed)
            
            self.iteration += 1
            rclpy.spin_once(self, timeout_sec=0)

def main(args=None):
    rclpy.init(args=args)
    controller = CameraController()
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()