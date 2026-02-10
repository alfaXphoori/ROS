#!/usr/bin/env python3

"""
Level 5.2: SLAM with Odometry (No GPS)
======================================
Creating maps without GPS, using wheel-based position calculation (Odometry)

Features:
1. Odometry: Calculate X, Y, Theta coordinates from Encoder and IMU
2. Lidar Mapping: Draw walls onto the grid (Occupancy Grid)
3. Console Visualizer: Display live map on screen
4. RViz Marker: Visualizes the robot body

Author: AI Assistant
"""

import rclpy
from rclpy.node import Node
from controller import Robot
import math
import sys
import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
from visualization_msgs.msg import Marker  # Import Marker
from tf2_ros import TransformBroadcaster
import time

class SlamOdometry(Node):
    def __init__(self):
        super().__init__('slam_odometry')
        
        # Initialize Webots
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())
        
        # ==================== CONFIGURATION PARAMETERS ====================
        
        # --- Robot Physical Parameters (Fixed to match World file) ---
        self.WHEEL_RADIUS = 0.08   # meters
        self.AXLE_LENGTH = 0.24    # meters
        
        # --- Speed Parameters ---
        self.MAX_SPEED = 4.0           
        self.TURN_SPEED_RATIO = 0.3    
        self.SLOW_SPEED_RATIO = 0.5    
        
        # --- Distance Parameters ---
        self.EMERGENCY_DISTANCE = 0.2 # meters - Too close! Reverse
        self.CRITICAL_DISTANCE = 0.3  # meters - Trigger Spin
        self.SAFE_DISTANCE = 0.6      # meters - Avoidance distance
        self.MAX_LIDAR_RANGE = 4.0     
        
        # --- Mapping Parameters ---
        self.MAP_SIZE = 200       # 200x200 grid
        self.RESOLUTION = 0.05    # 5cm per cell (10m x 10m total)
        
        # ==================================================================
        
        # --- Hardware Setup ---
        self.left_motor = self.robot.getDevice('left_motor')
        self.right_motor = self.robot.getDevice('right_motor')
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)
        
        # Encoders
        self.left_encoder = self.robot.getDevice('left_encoder')
        self.right_encoder = self.robot.getDevice('right_encoder')
        self.left_encoder.enable(self.timestep)
        self.right_encoder.enable(self.timestep)
        
        # Sensors
        self.lidar = self.robot.getDevice('lidar')
        self.lidar.enable(self.timestep)
        
        self.imu = self.robot.getDevice('imu')
        self.imu.enable(self.timestep)
        
        self.keyboard = self.robot.getKeyboard()
        self.keyboard.enable(self.timestep)
        
        # --- Mapping Config ---
        self.grid_map = np.zeros((self.MAP_SIZE, self.MAP_SIZE))
        
        # --- ROS2 Publishers ---
        self.scan_publisher = self.create_publisher(LaserScan, '/scan', 10)
        self.map_publisher = self.create_publisher(OccupancyGrid, '/map', 10)
        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)
        self.marker_publisher = self.create_publisher(Marker, '/robot', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # --- Odometry State ---
        self.pose = {'x': 0.0, 'y': 0.0, 'theta': 1.5708}  # Start facing North
        self.last_enc_l = 0.0
        self.last_enc_r = 0.0
        
        # --- State ---
        self.auto_mode = False
        self.iteration_count = 0
        
        # Stuck Logic
        self.stuck_timer = 0
        self.stuck_action = "NONE" # NONE, LEFT, RIGHT, BACK
        
        self.run()

    def update_odometry(self):
        """Calculate robot position from wheel rotation"""
        curr_enc_l = self.left_encoder.getValue()
        curr_enc_r = self.right_encoder.getValue()
        
        dl = (curr_enc_l - self.last_enc_l) * self.WHEEL_RADIUS
        dr = (curr_enc_r - self.last_enc_r) * self.WHEEL_RADIUS
        
        self.last_enc_l = curr_enc_l
        self.last_enc_r = curr_enc_r
        
        d_center = (dl + dr) / 2.0
        d_theta = (dr - dl) / self.AXLE_LENGTH
        
        # Use IMU for theta if available
        rpy = self.imu.getRollPitchYaw()
        imu_theta = rpy[2] 
        
        if imu_theta is not None and not math.isnan(imu_theta):
            self.pose['theta'] = imu_theta
        else:
            self.pose['theta'] += d_theta
        
        self.pose['x'] += d_center * math.cos(self.pose['theta'])
        self.pose['y'] += d_center * math.sin(self.pose['theta'])

    def world_to_map(self, wx, wy):
        if math.isnan(wx) or math.isnan(wy) or math.isinf(wx) or math.isinf(wy):
            return -1, -1
        
        mx = int((wx / self.RESOLUTION) + (self.MAP_SIZE / 2))
        my = int((wy / self.RESOLUTION) + (self.MAP_SIZE / 2))
        return mx, my

    def update_map(self, ranges):
        """Draw obstacles onto the map"""
        # Mark robot position (Trail)
        rx, ry = self.world_to_map(self.pose['x'], self.pose['y'])
        if 0 <= rx < self.MAP_SIZE and 0 <= ry < self.MAP_SIZE:
            self.grid_map[ry][rx] = 0.5 
            
        fov = self.lidar.getFov()
        h_res = self.lidar.getHorizontalResolution()
        
        for i, dist in enumerate(ranges):
            if dist == float('inf') or dist > self.MAX_LIDAR_RANGE:
                continue
            
            # Correct Angle Calculation (Match RViz)
            angle_rel = -((i / (h_res - 1)) * fov - (fov / 2))
            angle_global = self.pose['theta'] + angle_rel
            
            obs_x = self.pose['x'] + dist * math.cos(angle_global)
            obs_y = self.pose['y'] + dist * math.sin(angle_global)
            
            mx, my = self.world_to_map(obs_x, obs_y)
            if 0 <= mx < self.MAP_SIZE and 0 <= my < self.MAP_SIZE:
                self.grid_map[my][mx] = 1.0

    def auto_navigate(self, ranges):
        if not ranges: return 0.0, 0.0
        
        # --- Handle Timer-based actions ---
        if self.stuck_timer > 0:
            self.stuck_timer -= 1
            if self.stuck_action == "BACK":
                return -self.MAX_SPEED * 0.5, -self.MAX_SPEED * 0.5
            elif self.stuck_action == "LEFT":
                return -self.MAX_SPEED * 0.6, self.MAX_SPEED * 0.6
            elif self.stuck_action == "RIGHT":
                return self.MAX_SPEED * 0.6, -self.MAX_SPEED * 0.6
        
        n = len(ranges)
        # Precise Zones (based on Webots standard Lidar mapping)
        # 0 to 2PI. Mapping: Index 0 = Back-Left -> Middle = Front -> End = Back-Right
        # Wait, if angle_rel = -(i...), then:
        # i=0 -> +PI (Back Left)
        # i=N/4 -> +PI/2 (Left)
        # i=N/2 -> 0 (Front)
        # i=3N/4 -> -PI/2 (Right)
        # i=N -> -PI (Back Right)
        
        idx_front = n // 2
        idx_left = n // 4
        idx_right = 3 * n // 4
        
        # Sector width (e.g., +/- 30 degrees)
        width = n // 12
        
        def get_min_in_sector(center_idx, width):
            start = max(0, center_idx - width)
            end = min(n, center_idx + width)
            sector = ranges[start:end]
            return min([r for r in sector if r != float('inf')], default=10.0)

        min_front = get_min_in_sector(idx_front, width)
        min_left = get_min_in_sector(idx_left, width)
        min_right = get_min_in_sector(idx_right, width)
        
        # --- Decision Logic ---
        
        # 1. EMERGENCY: Too close -> Reverse and Turn
        if min_front < self.EMERGENCY_DISTANCE:
            self.stuck_timer = 15 # Back up for ~0.5s
            self.stuck_action = "BACK"
            return -self.MAX_SPEED * 0.5, -self.MAX_SPEED * 0.5

        # 2. OBSTACLE AHEAD: Turn towards open space
        if min_front < self.CRITICAL_DISTANCE:
            self.stuck_timer = 20 # Spin for ~0.6s
            if min_left > min_right:
                self.stuck_action = "LEFT"
                return -self.MAX_SPEED * 0.6, self.MAX_SPEED * 0.6
            else:
                self.stuck_action = "RIGHT"
                return self.MAX_SPEED * 0.6, -self.MAX_SPEED * 0.6

        # 3. MILD AVOIDANCE (Steer while moving)
        if min_front < self.SAFE_DISTANCE:
            if min_left > min_right:
                return self.MAX_SPEED * 0.3, self.MAX_SPEED * 0.8 # Bear Left
            else:
                return self.MAX_SPEED * 0.8, self.MAX_SPEED * 0.3 # Bear Right

        # 4. SIDE WALL FOLLOWING (Keep centered)
        # If too close to left wall, steer right
        if min_left < self.CRITICAL_DISTANCE:
             return self.MAX_SPEED * 0.8, self.MAX_SPEED * 0.6
        # If too close to right wall, steer left
        elif min_right < self.CRITICAL_DISTANCE:
             return self.MAX_SPEED * 0.6, self.MAX_SPEED * 0.8

        # 5. CLEAR -> FULL SPEED
        return self.MAX_SPEED, self.MAX_SPEED

    def publish_tf(self):
        now = self.get_clock().now().to_msg()
        
        # map -> odom
        t_map = TransformStamped()
        t_map.header.stamp = now
        t_map.header.frame_id = 'map'
        t_map.child_frame_id = 'odom'
        t_map.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t_map)

        # odom -> base_link
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.pose['x']
        t.transform.translation.y = self.pose['y']
        t.transform.translation.z = 0.1
        t.transform.rotation = self.euler_to_quaternion(0, 0, self.pose['theta'])
        self.tf_broadcaster.sendTransform(t)
        
        # base_link -> lidar_link
        t2 = TransformStamped()
        t2.header.stamp = now
        t2.header.frame_id = 'base_link'
        t2.child_frame_id = 'lidar_link'
        t2.transform.translation.x = 0.0
        t2.transform.translation.y = 0.0
        t2.transform.translation.z = 0.12
        t2.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t2)
    
    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return Quaternion(x=qx, y=qy, z=qz, w=qw)
    
    def publish_scan(self, ranges):
        if not ranges: return
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'lidar_link'
        scan.angle_min = math.pi
        scan.angle_max = -math.pi
        scan.angle_increment = -2.0 * math.pi / len(ranges)
        scan.range_min = 0.05
        scan.range_max = 4.0
        scan.ranges = [float(r) if r != float('inf') else float('inf') for r in ranges]
        self.scan_publisher.publish(scan)
    
    def publish_odometry(self):
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.pose['x']
        odom.pose.pose.position.y = self.pose['y']
        odom.pose.pose.position.z = 0.1
        odom.pose.pose.orientation = self.euler_to_quaternion(0, 0, self.pose['theta'])
        self.odom_publisher.publish(odom)
    
    def publish_robot_marker(self):
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "robot_body"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.scale.x = 0.4; marker.scale.y = 0.2; marker.scale.z = 0.1
        marker.color.a = 1.0; marker.color.r = 0.0; marker.color.g = 0.5; marker.color.b = 1.0
        marker.pose.orientation.w = 1.0
        self.marker_publisher.publish(marker)

    def publish_map(self):
        grid = OccupancyGrid()
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = 'map'
        grid.info.resolution = self.RESOLUTION
        grid.info.width = self.MAP_SIZE
        grid.info.height = self.MAP_SIZE
        grid.info.origin.position.x = -self.MAP_SIZE * self.RESOLUTION / 2.0
        grid.info.origin.position.y = -self.MAP_SIZE * self.RESOLUTION / 2.0
        grid.info.origin.position.z = 0.0
        grid.info.origin.orientation.w = 1.0
        
        data = []
        for y in range(self.MAP_SIZE):
            for x in range(self.MAP_SIZE):
                val = self.grid_map[y][x]
                if val == 1.0: data.append(100)
                elif val == 0.5: data.append(0)
                else: data.append(-1)
        grid.data = data
        self.map_publisher.publish(grid)
    
    def display_dashboard(self, ranges):
        sys.stdout.write("\033[H\033[J")
        mode = "AUTO" if self.auto_mode else "MANUAL"
        state = self.stuck_action if self.stuck_timer > 0 else "CRUISE"
        
        if ranges:
            n = len(ranges)
            idx_front = n // 2
            idx_left = n // 4
            idx_right = 3 * n // 4
            f = ranges[idx_front] if ranges[idx_front] != float('inf') else 9.9
            l = ranges[idx_left] if ranges[idx_left] != float('inf') else 9.9
            r = ranges[idx_right] if ranges[idx_right] != float('inf') else 9.9
        else: f=l=r=0.0
        
        print(f"{'='*50}")
        print(f" 🗺️  SLAM & NAV | Mode: {mode} | State: {state}")
        print(f"{'='*50}")
        print(f" Pos: X={self.pose['x']:.2f} Y={self.pose['y']:.2f}")
        print(f" Lidar: Front={f:.2f}m | Left={l:.2f}m | Right={r:.2f}m")
        print(f"{'-'*50}")
        print(" Controls: [W/A/S/D] Move | [M] Toggle Auto Mode")
        sys.stdout.flush()

    def run(self):
        try:
            while self.robot.step(self.timestep) != -1:
                self.update_odometry()
                ranges = self.lidar.getRangeImage()
                
                if ranges:
                    self.update_map(ranges)
                    self.publish_scan(ranges)
                
                self.publish_tf()
                self.publish_odometry()
                self.publish_robot_marker()
                
                if self.iteration_count % 5 == 0:
                    self.publish_map()
                
                self.display_dashboard(ranges)
                
                key = self.keyboard.getKey()
                left = 0.0
                right = 0.0
                
                if key == ord('M'):
                    self.auto_mode = not self.auto_mode
                
                if self.auto_mode:
                    left, right = self.auto_navigate(ranges)
                else:
                    if key == ord('W'): left, right = self.MAX_SPEED, self.MAX_SPEED
                    elif key == ord('S'): left, right = -self.MAX_SPEED, -self.MAX_SPEED
                    elif key == ord('A'): left, right = -self.MAX_SPEED*0.5, self.MAX_SPEED*0.5
                    elif key == ord('D'): left, right = self.MAX_SPEED*0.5, -self.MAX_SPEED*0.5
                
                self.left_motor.setVelocity(left)
                self.right_motor.setVelocity(right)
                
                self.iteration_count += 1
                rclpy.spin_once(self, timeout_sec=0)
                
        except KeyboardInterrupt:
            pass

def main(args=None):
    rclpy.init(args=args)
    controller = SlamOdometry()
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
