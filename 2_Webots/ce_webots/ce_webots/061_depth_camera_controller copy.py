#!/usr/bin/env python3

"""
Level 6.1: RGB-D Camera & Lidar Controller
==========================================
Control the RGBDBot and publish sensor data to ROS 2.

Features:
1.  Drive using Keyboard (W/A/S/D or Arrows) - Case Insensitive
2.  Publish RGB Camera (/camera/rgb/image_raw)
3.  Publish Depth Camera (/camera/depth/image_raw)
4.  Publish Lidar Scans (/scan)
5.  Max Speed defined in Python code
6.  [NEW] Publish TF and Odometry for RViz visualization

Author: AI Assistant
"""

import rclpy
from rclpy.node import Node
from controller import Robot, Keyboard
from sensor_msgs.msg import Image, LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster
import sys
import numpy as np
import math

class RGBDController(Node):
    def __init__(self):
        super().__init__('rgbd_controller')
        
        # Initialize Webots
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())
        
        # --- Configuration Parameters ---
        self.MAX_SPEED = 10.0 
        self.WHEEL_RADIUS = 0.08
        self.AXLE_LENGTH = 0.24
        
        self.get_logger().info(f'🚀 Max Speed set to: {self.MAX_SPEED} rad/s')
        
        # --- Hardware Setup ---
        self.left_motor = self.robot.getDevice('left_motor')
        self.right_motor = self.robot.getDevice('right_motor')
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)
        
        # Encoders & IMU (For Odometry/TF)
        self.left_encoder = self.robot.getDevice('left_encoder')
        self.right_encoder = self.robot.getDevice('right_encoder')
        self.left_encoder.enable(self.timestep)
        self.right_encoder.enable(self.timestep)
        
        self.imu = self.robot.getDevice('imu')
        self.imu.enable(self.timestep)
        
        self.keyboard = self.robot.getKeyboard()
        self.keyboard.enable(self.timestep)
        
        # --- Sensors ---
        self.camera_rgb = self.robot.getDevice('camera_rgb')
        self.camera_rgb.enable(self.timestep)
        
        self.camera_depth = self.robot.getDevice('camera_depth')
        self.camera_depth.enable(self.timestep)
        
        self.lidar = self.robot.getDevice('lidar')
        self.lidar.enable(self.timestep)
        self.lidar.enablePointCloud() 
        
        # --- ROS 2 Publishers ---
        self.rgb_pub = self.create_publisher(Image, '/camera/rgb/image_raw', 10)
        self.depth_pub = self.create_publisher(Image, '/camera/depth/image_raw', 10)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        
        # Add TF and Odom publishers
        self.tf_broadcaster = TransformBroadcaster(self)
        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)
        
        # --- Odometry State ---
        self.pose = {'x': 0.0, 'y': 0.0, 'theta': 1.5708} # Facing North
        self.last_enc_l = 0.0
        self.last_enc_r = 0.0
        
        self.run()

    def update_odometry(self):
        curr_l = self.left_encoder.getValue()
        curr_r = self.right_encoder.getValue()
        
        dl = (curr_l - self.last_enc_l) * self.WHEEL_RADIUS
        dr = (curr_r - self.last_enc_r) * self.WHEEL_RADIUS
        
        self.last_enc_l = curr_l
        self.last_enc_r = curr_r
        
        dist = (dl + dr) / 2.0
        d_theta = (dr - dl) / self.AXLE_LENGTH
        
        rpy = self.imu.getRollPitchYaw()
        if rpy and not math.isnan(rpy[2]):
            self.pose['theta'] = rpy[2]
        else:
            self.pose['theta'] += d_theta
            
        self.pose['x'] += dist * math.cos(self.pose['theta'])
        self.pose['y'] += dist * math.sin(self.pose['theta'])

    def euler_to_quaternion(self, r, p, y):
        qx = math.sin(r/2) * math.cos(p/2) * math.cos(y/2) - math.cos(r/2) * math.sin(p/2) * math.sin(y/2)
        qy = math.cos(r/2) * math.sin(p/2) * math.cos(y/2) + math.sin(r/2) * math.cos(p/2) * math.sin(y/2)
        qz = math.cos(r/2) * math.cos(p/2) * math.sin(y/2) - math.sin(r/2) * math.sin(p/2) * math.cos(y/2)
        qw = math.cos(r/2) * math.cos(p/2) * math.cos(y/2) + math.sin(r/2) * math.sin(p/2) * math.sin(y/2)
        return Quaternion(x=qx, y=qy, z=qz, w=qw)

    def publish_tf(self):
        now = self.get_clock().now().to_msg()
        q = self.euler_to_quaternion(0, 0, self.pose['theta'])
        
        # 1. map -> odom (Static)
        t_map = TransformStamped()
        t_map.header.stamp = now
        t_map.header.frame_id = 'map'
        t_map.child_frame_id = 'odom'
        t_map.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t_map)
        
        # 2. odom -> base_link (Dynamic)
        t_odom = TransformStamped()
        t_odom.header.stamp = now
        t_odom.header.frame_id = 'odom'
        t_odom.child_frame_id = 'base_link'
        t_odom.transform.translation.x = self.pose['x']
        t_odom.transform.translation.y = self.pose['y']
        t_odom.transform.translation.z = 0.1
        t_odom.transform.rotation = q
        self.tf_broadcaster.sendTransform(t_odom)
        
        # 3. base_link -> Sensor Links
        # Lidar
        t_lidar = TransformStamped()
        t_lidar.header.stamp = now
        t_lidar.header.frame_id = 'base_link'
        t_lidar.child_frame_id = 'lidar_link'
        t_lidar.transform.translation.z = 0.12
        t_lidar.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t_lidar)
        
        # RGB Camera
        t_rgb = TransformStamped()
        t_rgb.header.stamp = now
        t_rgb.header.frame_id = 'base_link'
        t_rgb.child_frame_id = 'camera_rgb_link'
        t_rgb.transform.translation.x = 0.15
        t_rgb.transform.translation.z = 0.12
        t_rgb.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t_rgb)
        
        # Depth Camera
        t_depth = TransformStamped()
        t_depth.header.stamp = now
        t_depth.header.frame_id = 'base_link'
        t_depth.child_frame_id = 'camera_depth_link'
        t_depth.transform.translation.x = 0.15
        t_depth.transform.translation.z = 0.12
        t_depth.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t_depth)

    def publish_rgb(self):
        img_data = self.camera_rgb.getImage()
        if img_data:
            msg = Image()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "camera_rgb_link"
            msg.height = self.camera_rgb.getHeight()
            msg.width = self.camera_rgb.getWidth()
            msg.encoding = "bgra8"
            msg.is_bigendian = False
            msg.step = 4 * msg.width
            msg.data = img_data
            self.rgb_pub.publish(msg)

    def publish_depth(self):
        depth_data = self.camera_depth.getRangeImage()
        if depth_data:
            msg = Image()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "camera_depth_link"
            msg.height = self.camera_depth.getHeight()
            msg.width = self.camera_depth.getWidth()
            msg.encoding = "32FC1"
            msg.is_bigendian = False
            msg.step = msg.width * 4
            
            # Robust conversion
            arr = np.array(depth_data, dtype=np.float32)
            # Ensure shape matches (sometimes Webots returns 1D list)
            if len(arr.shape) == 1:
                arr = arr.reshape((msg.height, msg.width))
            
            msg.data = arr.tobytes()
            self.depth_pub.publish(msg)
            return depth_data

    def publish_scan(self):
        ranges = self.lidar.getRangeImage()
        if ranges:
            msg = LaserScan()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "lidar_link"
            msg.angle_min = -3.14159
            msg.angle_max = 3.14159
            msg.angle_increment = 2.0 * 3.14159 / len(ranges)
            msg.time_increment = 0.0
            msg.scan_time = self.timestep / 1000.0
            msg.range_min = self.lidar.getMinRange()
            msg.range_max = self.lidar.getMaxRange()
            msg.ranges = [float(r) if r != float('inf') else float('inf') for r in ranges]
            self.scan_pub.publish(msg)

    def print_dashboard(self, depth_data, key):
        sys.stdout.write("\033[H\033[J")
        print("="*50)
        print(f"   👀 RGB-D CONTROL | Max Speed: {self.MAX_SPEED}")
        print("="*50)
        
        if depth_data:
            w = self.camera_depth.getWidth()
            h = self.camera_depth.getHeight()
            idx = (h // 2) * w + (w // 2)
            # Safety check index
            if idx < len(depth_data):
                center_dist = depth_data[idx]
                min_dist = min(depth_data)
                
                print(f"📏 Depth: Center={center_dist:.2f}m | Min={min_dist:.2f}m")
                
                bar_len = 20
                val = min(center_dist, 3.0) / 3.0
                fill = int(val * bar_len)
                bar = "█" * fill + "-" * (bar_len - fill)
                print(f"   Prox: [{bar}]")

        print("-" * 50)
        print("🎮 Controls: W/A/S/D or Arrows")
        print(f"   Last Key: {key}")
        print("="*50)
        sys.stdout.flush()

    def run(self):
        while self.robot.step(self.timestep) != -1:
            self.update_odometry()
            
            self.publish_rgb()
            depth = self.publish_depth()
            self.publish_scan()
            self.publish_tf() # Important for RViz
            
            key = self.keyboard.getKey()
            left = 0.0
            right = 0.0
            
            if key == ord('W') or key == ord('w') or key == Keyboard.UP:
                left, right = self.MAX_SPEED, self.MAX_SPEED
            elif key == ord('S') or key == ord('s') or key == Keyboard.DOWN:
                left, right = -self.MAX_SPEED, -self.MAX_SPEED
            elif key == ord('A') or key == ord('a') or key == Keyboard.LEFT:
                left, right = -self.MAX_SPEED*0.5, self.MAX_SPEED*0.5
            elif key == ord('D') or key == ord('d') or key == Keyboard.RIGHT:
                left, right = self.MAX_SPEED*0.5, -self.MAX_SPEED*0.5
            
            self.left_motor.setVelocity(left)
            self.right_motor.setVelocity(right)
            
            self.print_dashboard(depth, key)
            rclpy.spin_once(self, timeout_sec=0)

def main(args=None):
    rclpy.init(args=args)
    controller = RGBDController()
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()