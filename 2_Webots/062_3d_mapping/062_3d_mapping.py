#!/usr/bin/env python3

"""
Level 6.2: 3D Mapping (Point Cloud Generation) - Auto Spin
==========================================================
Convert RGB-D data into 3D Point Cloud for 3D mapping

Features:
1.  publish_pointcloud: Create 3D Map from RGB + Depth
2.  Manual Control: Control robot with keyboard W/A/S/D
3.  Auto Spin: Press 'M' to make robot spin and scan automatically
4.  Visualization: View in RViz2 via /camera/points topic
5.  Robot Marker: Display 3D robot model in RViz (/robot_marker)

Author: AI Assistant
"""

import rclpy
from rclpy.node import Node
from controller import Robot, Keyboard
from sensor_msgs.msg import Image, LaserScan, PointCloud2, PointField
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
from visualization_msgs.msg import Marker 
from tf2_ros import TransformBroadcaster
import sys
import numpy as np
import math
import struct

class RGBDMapping(Node):
    def __init__(self):
        super().__init__('rgbd_mapping')
        
        # Initialize Webots
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())
        
        # --- Configuration ---
        self.MAX_SPEED = 4.0 
        self.WHEEL_RADIUS = 0.08
        self.AXLE_LENGTH = 0.24
        
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
        self.pcl_pub = self.create_publisher(PointCloud2, '/camera/points', 10)
        
        # Marker Publisher for Robot Body
        self.marker_pub = self.create_publisher(Marker, '/robot_marker', 10)
        
        self.tf_broadcaster = TransformBroadcaster(self)
        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)
        
        # --- State ---
        self.pose = {'x': 0.0, 'y': 0.0, 'theta': 1.5708} 
        self.last_enc_l = 0.0
        self.last_enc_r = 0.0
        self.auto_mode = False  # Start in Manual mode
        
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
        
        # map -> odom
        t_map = TransformStamped()
        t_map.header.stamp = now
        t_map.header.frame_id = 'map'
        t_map.child_frame_id = 'odom'
        t_map.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t_map)
        
        # odom -> base_link
        t_odom = TransformStamped()
        t_odom.header.stamp = now
        t_odom.header.frame_id = 'odom'
        t_odom.child_frame_id = 'base_link'
        t_odom.transform.translation.x = self.pose['x']
        t_odom.transform.translation.y = self.pose['y']
        t_odom.transform.translation.z = 0.1
        t_odom.transform.rotation = q
        self.tf_broadcaster.sendTransform(t_odom)
        
        # base_link -> camera_depth_link
        t_cam = TransformStamped()
        t_cam.header.stamp = now
        t_cam.header.frame_id = 'base_link'
        t_cam.child_frame_id = 'camera_depth_link'
        t_cam.transform.translation.x = 0.15
        t_cam.transform.translation.z = 0.12
        t_cam.transform.rotation.x = -0.5
        t_cam.transform.rotation.y = 0.5
        t_cam.transform.rotation.z = -0.5
        t_cam.transform.rotation.w = 0.5
        self.tf_broadcaster.sendTransform(t_cam)

        # base_link -> lidar_link
        t_lidar = TransformStamped()
        t_lidar.header.stamp = now
        t_lidar.header.frame_id = 'base_link'
        t_lidar.child_frame_id = 'lidar_link'
        t_lidar.transform.translation.z = 0.12
        t_lidar.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t_lidar)

    def publish_robot_markers(self):
        """Publish visual markers for the robot in RViz"""
        now = self.get_clock().now().to_msg()
        
        # 1. Robot Body (Blue Box)
        body = Marker()
        body.header.frame_id = "base_link"
        body.header.stamp = now
        body.ns = "robot_parts"
        body.id = 0
        body.type = Marker.CUBE
        body.action = Marker.ADD
        body.scale.x = 0.4; body.scale.y = 0.2; body.scale.z = 0.1
        body.color.r = 0.1; body.color.g = 0.5; body.color.b = 1.0; body.color.a = 1.0
        body.pose.orientation.w = 1.0
        self.marker_pub.publish(body)
        
        # 2. Lidar (Black Cylinder)
        lidar = Marker()
        lidar.header.frame_id = "base_link"
        lidar.header.stamp = now
        lidar.ns = "robot_parts"
        lidar.id = 1
        lidar.type = Marker.CYLINDER
        lidar.action = Marker.ADD
        lidar.scale.x = 0.16; lidar.scale.y = 0.16; lidar.scale.z = 0.06
        lidar.color.r = 0.1; lidar.color.g = 0.1; lidar.color.b = 0.1; lidar.color.a = 1.0
        lidar.pose.position.z = 0.12 # Matches .wbt translation
        lidar.pose.orientation.w = 1.0
        self.marker_pub.publish(lidar)
        
        # 3. Camera (Grey Box at Front)
        cam = Marker()
        cam.header.frame_id = "base_link"
        cam.header.stamp = now
        cam.ns = "robot_parts"
        cam.id = 2
        cam.type = Marker.CUBE
        cam.action = Marker.ADD
        cam.scale.x = 0.06; cam.scale.y = 0.1; cam.scale.z = 0.06
        cam.color.r = 0.3; cam.color.g = 0.3; cam.color.b = 0.3; cam.color.a = 1.0
        cam.pose.position.x = 0.15; cam.pose.position.z = 0.12
        cam.pose.orientation.w = 1.0
        self.marker_pub.publish(cam)

    def publish_scan(self, ranges):
        if not ranges: return
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'lidar_link'
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = 2.0 * math.pi / len(ranges)
        scan.range_min = 0.05
        scan.range_max = 4.0
        scan.ranges = [float(r) if r != float('inf') else float('inf') for r in ranges]
        self.scan_pub.publish(scan)

    def publish_pointcloud(self, depth_data, rgb_data):
        width = self.camera_depth.getWidth()
        height = self.camera_depth.getHeight()
        fov = self.camera_depth.getFov()
        f_x = width / (2 * math.tan(fov / 2))
        f_y = f_x
        c_x, c_y = width / 2, height / 2

        depth = np.array(depth_data, dtype=np.float32).reshape((height, width))
        rgb_arr = np.frombuffer(rgb_data, dtype=np.uint8).reshape((height, width, 4))
        
        valid_mask = (depth > 0.1) & (depth < 3.0)
        u, v = np.meshgrid(np.arange(width), np.arange(height))
        z = depth
        x = (u - c_x) * z / f_x
        y = (v - c_y) * z / f_y
        
        r = rgb_arr[:, :, 2].astype(np.uint32)
        g = rgb_arr[:, :, 1].astype(np.uint32)
        b = rgb_arr[:, :, 0].astype(np.uint32)
        rgb_packed = (r << 16) | (g << 8) | b
        
        x_flat = x[valid_mask]
        y_flat = y[valid_mask]
        z_flat = z[valid_mask]
        rgb_flat = rgb_packed[valid_mask]
        
        points = np.zeros(x_flat.shape[0], dtype=[
            ('x', np.float32), ('y', np.float32), ('z', np.float32), ('rgb', np.float32)])
            
        points['x'] = x_flat
        points['y'] = y_flat
        points['z'] = z_flat
        points['rgb'] = rgb_flat.view(np.float32)
        
        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_depth_link"
        msg.height = 1
        msg.width = points.shape[0]
        msg.fields = [
            PointField(name='x', offset=0, datatype=7, count=1),
            PointField(name='y', offset=4, datatype=7, count=1),
            PointField(name='z', offset=8, datatype=7, count=1),
            PointField(name='rgb', offset=12, datatype=7, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step = 16 * points.shape[0]
        msg.is_dense = True
        msg.data = points.tobytes()
        self.pcl_pub.publish(msg)

    def print_dashboard(self, key):
        sys.stdout.write("\033[H\033[J")
        mode_str = "🤖 AUTO SPIN" if self.auto_mode else "🎮 MANUAL"
        print("="*50)
        print(f"   🧊 3D MAPPING | PointCloud2 ({mode_str})")
        print("="*50)
        print(f" Position: X={self.pose['x']:.2f} Y={self.pose['y']:.2f}")
        print("-" * 50)
        print("🎮 Controls: W/A/S/D or Arrows | [M] Toggle Auto Spin")
        print("📡 Topics:")
        print("   - /camera/points (Add this in RViz!)")
        print("   - /robot_marker  (Robot Visual)")
        print("="*50)
        sys.stdout.flush()

    def run(self):
        while self.robot.step(self.timestep) != -1:
            self.update_odometry()
            self.publish_tf()
            self.publish_robot_markers()
            
            # Get Data
            rgb_data = self.camera_rgb.getImage()
            depth_data = self.camera_depth.getRangeImage()
            lidar_data = self.lidar.getRangeImage()
            
            if rgb_data and depth_data:
                self.publish_pointcloud(depth_data, rgb_data)
            
            if lidar_data:
                self.publish_scan(lidar_data)
            
            # Control
            key = self.keyboard.getKey()
            if key == ord('M'):
                self.auto_mode = not self.auto_mode
            
            left = 0.0
            right = 0.0
            
            if self.auto_mode:
                # Auto Spin logic
                left = -2.0
                right = 2.0
            else:
                if key == ord('W') or key == ord('w') or key == Keyboard.UP:
                    left, right = 5.0, 5.0
                elif key == ord('S') or key == ord('s') or key == Keyboard.DOWN:
                    left, right = -5.0, -5.0
                elif key == ord('A') or key == ord('a') or key == Keyboard.LEFT:
                    left, right = -2.5, 2.5
                elif key == ord('D') or key == ord('d') or key == Keyboard.RIGHT:
                    left, right = 2.5, -2.5
            
            self.left_motor.setVelocity(left)
            self.right_motor.setVelocity(right)
            
            self.print_dashboard(key)
            rclpy.spin_once(self, timeout_sec=0)

def main(args=None):
    rclpy.init(args=args)
    controller = RGBDMapping()
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()