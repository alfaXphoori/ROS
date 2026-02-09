#!/usr/bin/env python3

"""
Level 4.2: Line Follower (Camera)
=================================
หุ่นยนต์เดินตามเส้นสีดำบนพื้นขาวโดยใช้กล้อง

Logic:
1. อ่านภาพจากกล้อง (Grayscale)
2. ตัดภาพเฉพาะส่วนล่าง (ROI - Region of Interest) เพื่อดูเส้นที่อยู่ใกล้ตัว
3. หาตำแหน่งของ pixel สีดำ (Dark pixels)
4. คำนวณจุดศูนย์กลางของเส้น (Centroid)
5. คำนวณ Error = (จุดกึ่งกลางภาพ - จุดศูนย์กลางเส้น)
6. ปรับความเร็วมอเตอร์ซ้าย/ขวา เพื่อเลี้ยงให้ Error เป็น 0 (P-Controller)

Author: AI Assistant
"""

import rclpy
from rclpy.node import Node
from controller import Robot
import sys

class LineFollower(Node):
    def __init__(self):
        super().__init__('line_follower')
        
        # Initialize Webots
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())
        
        # --- Motor Setup ---
        self.left_motor = self.robot.getDevice('left_motor')
        self.right_motor = self.robot.getDevice('right_motor')
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)
        
        # --- Camera Setup ---
        self.camera = self.robot.getDevice('camera')
        self.camera.enable(self.timestep)
        
        self.width = self.camera.getWidth()
        self.height = self.camera.getHeight()
        
        self.get_logger().info(f'📷 Line Follower Started: {self.width}x{self.height}')
        
        self.run()

    def process_image_and_control(self):
        # 1. รับภาพ (Raw Bytes -> BGRA)
        image = self.camera.getImage()
        if not image: return

        # 2. ตั้งค่าการตรวจจับ (ROI: แถวล่างๆ ของภาพ)
        # เราจะสแกนเฉพาะแถวที่ 80 ถึง 100 (จากความสูง 120) เพื่อมองเส้นข้างหน้า
        scan_line_y = int(self.height * 0.8) 
        
        black_pixel_count = 0
        sum_x = 0
        
        # Threshold สีดำ (ค่าสีต่ำกว่านี้ถือว่าเป็นเส้น)
        BLACK_THRESHOLD = 80
        
        # สร้าง Visual Log (string) เพื่อแสดงผลหน้าจอแบบ ASCII Art
        line_visual = ["."] * self.width
        
        # 3. สแกน Pixel ในแถวที่กำหนด
        # image data เป็น 1D array: [B, G, R, A, B, G, R, A, ...]
        for x in range(self.width):
            # คำนวณ index ใน array
            # pixel_index = (y * width + x) * 4
            pixel_idx = (scan_line_y * self.width + x) * 4
            
            # อ่านค่าสี (เฉลี่ย RGB เพื่อดูความสว่าง/Greyscale)
            blue = image[pixel_idx]
            green = image[pixel_idx + 1]
            red = image[pixel_idx + 2]
            gray = (int(red) + int(green) + int(blue)) / 3
            
            # ตรวจสอบว่าเป็นสีดำหรือไม่
            if gray < BLACK_THRESHOLD:
                black_pixel_count += 1
                sum_x += x
                line_visual[x] = "#" # สัญลักษณ์แทนเส้นดำ
        
        # 4. คำนวณการควบคุม
        base_speed = 4.0
        turn_correction = 0.0
        error = 0
        state = "SEARCHING"
        
        if black_pixel_count > 0:
            # เจอเส้น! คำนวณจุดกึ่งกลาง (Centroid)
            line_center_x = sum_x / black_pixel_count
            
            # จุดกึ่งกลางภาพ
            image_center_x = self.width / 2
            
            # Error = ระยะห่างจากจุดกึ่งกลางภาพ (เป็น + ถ้าเส้นอยู่ขวา, - ถ้าเส้นอยู่ซ้าย)
            error = line_center_x - image_center_x
            
            # --- P-Controller (Proportional Control) ---
            # ปรับค่า Kp เพื่อเปลี่ยนความไวในการเลี้ยว
            Kp = 0.15 
            turn_correction = error * Kp
            
            state = "TRACKING"
            
            # วาดจุดกึ่งกลางเส้นลงใน visual log
            center_idx = int(line_center_x)
            if 0 <= center_idx < self.width:
                line_visual[center_idx] = "O" # จุดที่หุ่นยนต์เล็ง
        else:
            # ไม่เจอเส้น (อาจจะหลุดโค้ง หรือจบเส้นทาง)
            # ให้หมุนวนหาเส้น (Spin)
            turn_correction = 3.0 # หมุนขวา
            base_speed = 0.0
            state = "LOST LINE"

        # 5. สั่งมอเตอร์ (Differential Drive)
        left_speed = base_speed + turn_correction
        right_speed = base_speed - turn_correction
        
        # Limit speed
        left_speed = max(min(left_speed, 10.0), -10.0)
        right_speed = max(min(right_speed, 10.0), -10.0)
        
        self.left_motor.setVelocity(left_speed)
        self.right_motor.setVelocity(right_speed)
        
        # 6. แสดงผล Dashboard
        self._print_dashboard(line_visual, error, state, left_speed, right_speed)

    def _print_dashboard(self, line_visual, error, state, l_spd, r_spd):
        """แสดงผลแบบ Real-time"""
        sys.stdout.write("\033[H\033[J") # Clear screen
        
        # ย่อ Visual ให้พอดีหน้าจอ (ถ้ากว้างเกินไป)
        display_str = "".join(line_visual)
        if len(display_str) > 60:
            step = len(display_str) // 60
            display_str = display_str[::step]
            
        print(f"{'='*60}")
        print(f"   🛤️  LINE FOLLOWER DASHBOARD")
        print(f"{'='*60}\n")
        
        print(f"CAMERA VIEW (Thresholded):")
        print(f"[{display_str}]")
        print(f" {' '*int(len(display_str)/2)}^ (Center)")
        print(f"\n")
        
        print(f"📊 DATA:")
        print(f"   Status:      {state}")
        print(f"   Error:       {error:.2f} pixels")
        print(f"   Motors:      L={l_spd:.2f} | R={r_spd:.2f}")
        
        print(f"\n{'-'*60}")
        print(f"Tip: If robot loses line, adjust Kp or speed.")
        print(f"{'='*60}")
        sys.stdout.flush()

    def run(self):
        while self.robot.step(self.timestep) != -1:
            self.process_image_and_control()
            rclpy.spin_once(self, timeout_sec=0)

def main(args=None):
    rclpy.init(args=args)
    controller = LineFollower()
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()