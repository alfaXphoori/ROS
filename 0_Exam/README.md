# 📝 สอบปฏิบัติ - ROS2 & Webots

> **เวลาสอบ: 3 ชั่วโมง**

---

## 📋 ข้อกำหนดทั่วไป

| รายการ | รายละเอียด |
|--------|-----------|
| ภาษา | **Python เท่านั้น** |
| Package | `exam_ชื่อนักศึกษา` เช่น `exam_phoori` |
| Node file | `exam_ชื่อนักศึกษา_node.py` เช่น `exam_phoori_node.py` |
| World file | `exam_world.wbt` |

---

## 🎯 ภาระงาน (Tasks)

### Task 1 — สร้าง ROS2 Package

สร้าง package ใหม่ชื่อ `exam_ชื่อนักศึกษา` (แทนที่ด้วยชื่อจริง เช่น `exam_phoori`)

โครงสร้าง package ที่ต้องใช้:

```
exam_phoori/
├── exam_phoori/
│   ├── __init__.py
│   └── exam_phoori_node.py        ← ไฟล์หลัก (Node เดียว ทำทุก task)
├── package.xml
├── setup.py
└── setup.cfg
```

---

### Task 2 — ควบคุมหุ่นยนต์ + SLAM + กล้อง

เขียน node ใน `exam_ชื่อนักศึกษา_node.py` ที่ทำงานพร้อมกัน **3 อย่าง**:

1. **ควบคุมหุ่นยนต์** ให้เคลื่อนที่สำรวจพื้นที่ใน `exam_world.wbt` (keyboard หรือ autonomous)
2. **ทำแผนที่ SLAM** โดยใช้ Lidar สร้าง Occupancy Grid และ publish ผ่าน ROS2
3. **เปิดกล้อง** และ publish ภาพจากกล้องหุ่นยนต์ผ่าน topic `/image_raw`

## �️ ตัวอย่างผลงานที่ถูกต้อง

### ตัวอย่างที่ 1 — แผนที่ SLAM + หุ่นยนต์ใน Webots

![example_slam](imgs/example_slam.png)

> แผนที่ Occupancy Grid ที่ถูกต้องจะเห็นผนังห้อง (สีดำ) และพื้นที่เปิด (สีเทา) ในหน้าต่าง RViz  
> พร้อมกับหุ่นยนต์ที่แสดงอยู่ใน Webots และ Camera view

---

### ตัวอย่างที่ 2 — topic `/robot_name`

![example_topic](imgs/example_topic.png)

> ผลลัพธ์ที่ถูกต้องจากคำสั่ง `ros2 topic echo /robot_name`  
> ข้อมูลจะแสดงชื่อ นามสกุล และรหัสนักศึกษา ซ้ำทุก ๆ cycle

---

## �📸 การส่งงาน

| # | สิ่งที่ต้องส่ง | รายละเอียด |
|---|--------------|-----------|
| 1 | **ภาพแผนที่ SLAM** | ภาพ screenshot แผนที่ที่ทำเสร็จแล้ว + ภาพหุ่นยนต์ใน Webots |
| 2 | **ภาพ topic /robot_name** | ภาพ screenshot แสดงผลคำสั่ง `ros2 topic echo /robot_name` |
| 3 | **ZIP file** | บีบอัด folder `exam_ชื่อนักศึกษา` → `exam_ชื่อนักศึกษา.zip` เช่น `exam_phoori.zip` |

### วิธีสร้าง ZIP file

```bash
cd ~/ros2_ws/src
zip -r exam_phoori.zip exam_phoori/
```

---

## ✅ เกณฑ์การให้คะแนน

| หัวข้อ | คะแนน |
|--------|-------|
| สร้าง package และ node ถูกต้องตามชื่อที่กำหนด | 2 |
| Node ทำงานได้ (ไม่ error เมื่อ run) | 2 |
| หุ่นยนต์เคลื่อนที่ได้ใน Webots | 4 |
| publish `/scan` และสร้างแผนที่ SLAM ได้ | 6 |
| publish `/image_raw` จากกล้องได้ | 3 |
| publish `/robot_name` มีชื่อ-นามสกุล-รหัส นศ | 3 |
| **รวม** | **20** |


## 📤 ลิงก์ส่งงาน

> **[👉 คลิกที่นี่เพื่อส่งงาน](https://docs.google.com/forms/d/e/1FAIpQLScJNaKMrzGsNUltcTbqbFs8zb7nuiC_8Rb9BtSzjeCdtY4VQQ/viewform?usp=dialog)**

---

*Good Luck! 🤖*
