# 🤖 สอบปฏิบัติ — ROS2 & Webots

---

<div align="center">

| 🕐 เวลาสอบ | 📅 วันที่ | ⏰ กำหนดส่ง |
|:-----------:|:--------:|:-----------:|
| **3 ชั่วโมง** | **10 / 03 / 2026** | **ไม่เกิน 12:15 น.** |

</div>

> ⚠️ **งานที่ส่งหลัง 12:15 น. จะไม่ได้รับการตรวจ**

---

## 📋 ข้อกำหนดทั่วไป

| รายการ | รายละเอียด |
|:------:|-----------|
| 🐍 ภาษา | **Python เท่านั้น** |
| 📦 Package | `exam_ชื่อนักศึกษา` เช่น `exam_phoori` |
| 📄 Node file | `exam_ชื่อนักศึกษา_node.py` เช่น `exam_phoori_node.py` |
| 🌍 World file | [📥 ดาวน์โหลด exam_world.wbt](imgs/exam_world.wbt) |

---

## 🎯 ภาระงาน (Tasks)

### ✅ Task 1 — สร้าง ROS2 Package

สร้าง package ใหม่ชื่อ `exam_ชื่อนักศึกษา` (เช่น `exam_phoori`)

```
exam_phoori/
├── exam_phoori/
│   ├── __init__.py
│   └── exam_phoori_node.py    ← ไฟล์หลัก (Node เดียว ทำทุก task)
├── package.xml
├── setup.py
└── setup.cfg
```

---

### ✅ Task 2 — ควบคุมหุ่นยนต์ + SLAM + กล้อง

เขียน node ใน `exam_ชื่อนักศึกษา_node.py` ให้ทำงานพร้อมกัน **3 อย่าง**:

| # | งาน | รายละเอียด |
|---|-----|-----------|
| 1 | 🕹️ **ควบคุมหุ่นยนต์** | เคลื่อนที่สำรวจพื้นที่ใน `exam_world.wbt` (keyboard หรือ autonomous) |
| 2 | 🗺️ **SLAM** | ใช้ Lidar สร้าง Occupancy Grid และ publish ผ่าน ROS2 |
| 3 | 📷 **กล้อง** | publish ภาพจากกล้องหุ่นยนต์ผ่าน topic `/image_raw` |

---

### ✅ Task 3 — Publisher `/robot_name`

publish ข้อมูลส่วนตัวผ่าน topic `/robot_name` รูปแบบ:

```
Name: [ชื่อ นามสกุล]  ID: [รหัสนักศึกษา]
```

ตรวจสอบด้วย:
```bash
ros2 topic echo /robot_name
```

---

## 🖼️ ตัวอย่างผลงานที่ถูกต้อง

### แผนที่ SLAM + หุ่นยนต์ใน Webots
<img src="imgs/example_slam.png" width="600"/>

### topic `/robot_name`
<img src="imgs/example_topic.png" width="500"/>

---

## 📸 การส่งงาน

| # | สิ่งที่ต้องส่ง | รายละเอียด |
|:-:|--------------|-----------|
| 1 | 🗺️ **ภาพแผนที่ SLAM** | screenshot แผนที่ + ภาพหุ่นยนต์ใน Webots |
| 2 | 📡 **ภาพ topic /robot_name** | screenshot ผลคำสั่ง `ros2 topic echo /robot_name` |
| 3 | 🗜️ **ZIP file** | `exam_ชื่อนักศึกษา.zip` เช่น `exam_phoori.zip` |

**วิธีสร้าง ZIP:**
```bash
cd ~/ros2_ws/src
zip -r exam_phoori.zip exam_phoori/
```

---

## ✅ เกณฑ์การให้คะแนน

| หัวข้อ | คะแนน |
|--------|:-----:|
| สร้าง package และ node ถูกต้องตามชื่อที่กำหนด | 2 |
| Node ทำงานได้ (ไม่ error เมื่อ run) | 2 |
| หุ่นยนต์เคลื่อนที่ได้ใน Webots | 4 |
| publish `/scan` และสร้างแผนที่ SLAM ได้ | 6 |
| publish `/image_raw` จากกล้องได้ | 3 |
| publish `/robot_name` มีชื่อ-นามสกุล-รหัส นศ | 3 |
| **รวม** | **20** |

---

## 📤 ส่งงาน

> ### [👉 คลิกที่นี่เพื่อส่งงาน](https://docs.google.com/forms/d/e/1FAIpQLScJNaKMrzGsNUltcTbqbFs8zb7nuiC_8Rb9BtSzjeCdtY4VQQ/viewform?usp=dialog)
> ⏰ ส่งได้ **ไม่เกิน 12:15 น.** วันที่ 10/03/2026

---

*Good Luck! 🤖*
