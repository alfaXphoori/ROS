# Project Title
ROS2 Jazzy Jalisco /Ubuntu 24_04
## Authors

- [@alfaXphoori](https://www.github.com/alfaXphoori)


## Environment Variables

Ros2 ที่จะใช้านจะทำงานบน Visual Machine โดยทำการติดตั้ง
- VMware Workstation Pro
Ros2 เลือกใช้การทำงานบนระบบปัฎิบัติการ Linux เลือกใช้งานเป็น Ubuntu
- Install Ubuntu 24.04.3 LTS


## Installation

การติดดตั้ง ROS2 ตาม Link 

https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html

ใช้คำสั่ง ls -a เพื่อค้นหาตำแหน่งของ .bashrc
```bash
ls -a
```

เปิด .bashrc โดยโปรแกรม Nano
```bash
nano .bashrc
```

เพิ่มคำสั่งนีใน บรรทัดสุดท้าย
```bash
source /opt/ros/jazzy/setup.bash
```
ทดสอบโดยเปิด Terminal ใหม่พร้อมใช้คำสั่ง
```bash
ros2 --help
```

ยินดีด้วยติดตั้ง ROS2 สำเร็จแล้ว

## Install Colcon
เริ่มติดตั้ง Colcon
เปิด terminal เข้า source ทดสอบว่าต้องติดตั้ง ROS2 เรียบร้อยแล้ว
ติดตั้งโปรแกรม Colcon
```bash
sudo apt install python3-colcon-common-extensions
```
ทดสอบการติดตั้ง Colocon โดยเข้าไปที่ Path จะพบ ไฟล์ colcon-argcomplete.bash
```bash
cd /usr/share/colcon_argcomplete/hook/
```
จากนั้นทำการตั้งค่า ./bashrc โดยเพิ่ม 3 บรรทัดนี้เข้าไปยัง ท้ายสุดของ ไฟล์

- source /opt/ros/iron/setup.bash
- source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
- source ~/ros2_ws/install/setup.bash

```bash
gedit ~/.bashrc
```
เสร็จสิ้นการติดตั้ง Colcon ใน ROS2 สำหรับสร้าง package ใหม่

## Install Visual Studio Code
ทำการติดตั้ง vscode ใน Ubuntu เพื่อให้เขียน Code ได้ง่ายขึ้น\
หลังจากเข้ามา vscode ทำการติดตั้ง Extensions ที่ publish by Microsoft
- c++
- python
- cmake
- cmake tools
- xml
- xml tools
- ros

ติดตั้ง python3-pip สำหรับการ build package

```bash
sudo apt install python3-pip
```

## Create First Package
เมื่อเราติดตั้ง ROS2 และ Tools ต่างๆจนครบ \
เราจะพร้อมที่จะสร้าง Package เพื่อใช้งานแล้วตอนนี้ \
เริ่มต้นด้วยการเปิด terminal ขึ้นมา ทำการสร้าง Folder ros2_ws สำหรับ ROS2 

```bash
mkdir ros2_ws
colcon build

cd ros2_ws
mkdir src
```

จากนั้นทำการสร้าง Package จากคำสั่ง Ros create โดยสร้างไว้ที่ Folder src\
โดย Package ที่สร้างขึ้นมาจะมีชื่อว่า ce_robot

```bash
cd src
ros2 pkg create ce_robot --build-type ament_python --dependencies rclpy
```

จากนั้นเข้าไปยัง Folder ce_robot/ce_robot

```bash
cd ce_robot/ce_robot
```
เมื่อได้ Package แล้วทำการสร้าง Node เพื่อทำงานใน Ros2
สร้าง file python ที่มีชื่อว่า first_node.py พร้อมเปลี่ยน Permission file เป็น +x 
```bash
touch first_node.py
chmod +x first_node.py
```

จากนั้นทำการเขียน Code ภาษา python เมื่อเสร็จแล้วทำการทดสอบ file โดยใช้คำสั่ง 
```bash
./first_node.py
```

แก้ไข file package.xml โดยเพิ่ม code ส่วน library\
เพิ่ม code ภายใต้ 'console_scripts': [ ] ของ file setup.py
- "first_node = ce_robot.first_node:main",

เมื่อ Code ไม่มี error แล้วต้องทำการ Colcon build เพื่อให้ Package \
ที่เราสร้างขึ้นสามารถใช้งานผ่าน คำสั่ง ros2 run ได้
```bash
cd ~/ros2_ws
colcon build 
```

เมื่อทำการ colcon build สำเร็จ ทำการทดสอบการ ทำงาน Package ที่สร้างขึ้นโดย \
เปิด terminal เข้า source ./bashrc แล้วใช้คำสั่ง ros2 run package ที่สร้างขึ้น
```bash
source ~/.bashrc
ros2 run ce_robot first_node
```

เสร็จสิ้นสำหรับการสร้าง Package แรก ใน ros2

Directory Tree
```bash
|--ros2_ws
   |--build
   |--intstall
   |--log
   |--src
      |--ce_robot
         |--ce_robot
            |--first_node.py
          
```
