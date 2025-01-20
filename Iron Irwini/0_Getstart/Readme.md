# Project Title

ROS2 Ubuntu 22_04

We chose Ros2 Iron Irwini to use.
## Authors

- [@alfaXphoori](https://www.github.com/alfaXphoori)


## Environment Variables

Ros2 ที่จะใช้านจะทำงานบน Visual Machine โดยทำการติดตั้ง
- VMware Workstation Pro
Ros2 เลือกใช้การทำงานบนระบบปัฎิบัติการ Linux เลือกใช้งานเป็น Ubuntu
- Install Ubuntu 22.04.3 LTS (Jammy Jellyfish)


## Installation

การติดดตั้ง ROS2 เริ่มด้วยการเปลี่ยน Locale ของเครื่อง Ubuntu \
ทำการเปิด Terminal และใส่คำสั่งดังต่อไปนี้

```bash
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify setting
```

ติดตั้ง repository ใน Ubuntu 

```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
```

ติดตั้ง key สำหรับ ROS
```bash
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```

เพิ่ม repository ใน sources list
```bash 
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

ติดตั้ง Ros development tools 
```bash 
sudo apt update && sudo apt install ros-dev-tools
```

Update & upgrade Ubuntu
```bash
sudo apt update
sudo apt upgrade
```

ติดตั้ง ROS2 Iron desktop
```bash
sudo apt install ros-iron-desktop
```

ติดตั้ง ROS2 Iron base
```bash
sudo apt install ros-iron-ros-base
```

ยินดีด้วยติดตั้ง ROS2 สำเร็จแล้ว
## Usage/Examples

เมื่อติดตั้ง ROS2 เสร็จเรียบร้อยแล้ว ทำการทดสอบการทำงาน\
เริ่มเปิด terminal ใน Ubuntu ทำการเข้า source ตามคำสั่ง
```bash
source /opt/ros/iron/setup.bash
```
ทดสอบว่ามีการติดตั้ง ROS2 หรือไม่
```bash
ros2
printenv | grep -i ROS
```

ทดสอบการทำงานของ Node ใน ROS2\
โดยเราเลือกใช้ ภาษา Python3 ในการเขียนโปรแกรมใน ROS2\
เปิด terminal ทำการเข้า source แล้ว run node ตัวอย่าง demo_nodes_py talker
```bash
source /opt/ros/iron/setup.bash
ros2 run demo_nodes_py talker
```

เปิด terminal อีกหน้าต่าง ทำการเข้า source แล้ว run node ตัวอย่าง demo_nodes_py listener
```bash
source /opt/ros/iron/setup.bash
ros2 run demo_nodes_py listener
```

หลังจาก run node ทดสอบทั้ง 2 แล้ว\
ทำการเปิด terminal อีกหน้าต่าง เพื่อทำการดูการเชื่อมต่อของ node ทั้งสอง
```bash
source /opt/ros/iron/setup.bash
rqt_graph
```
ยินดีกับการ run node ทดสอบใน Ros2 ได้สำเร็จ


## Turtlesim
สำหรับ ROS2 ในการทดสอบการทำงาน จะมี Simulation ให้ใช้งาน มีชื่อว่า Turtlesim\
ก่อนอื่นทำการติดตั้ง Turtlesim เพื่อใช้งาน
```bash
sudo apt update
sudo apt install ros-iron-turtlesim 
ros2 pkg executables turtlesim
```

ทดสอบการทำงานของ Turtlesim เปิด terminal เข้า source แล้ว run
```bash
source /opt/ros/iron/setup.bash
ros2 run turtlesim turtlesim_node
```

เปิด terminal อีกหน้าต่าง พร้อมเข้า source แล้ว run
```bash
source /opt/ros/iron/setup.bash
ros2 run turtlesim turtle_teleop_key
```

หลังจาก run Ros2 turtlesim_node ทั้ง 2 แล้ว\
เราจะสามารถควบคุมตัวเต่าใน turtlesim โดยใช้ลูกศรภายใน Keyboard  
## Install Colcon
หลังจากที่เรา ติตตั้งและ ทดสอบ Ros2 เสร็จสิ้นหมดแล้ว\
ในสร้าง Package สำหรับใช้งานเองใน Ros2 ต้องติดตั้งโปรแกรม Colcon \
สำหรับการ build Package ที่สร้างขึ้น

เริ่มติดตั้ง Colcon
เปิด terminal เข้า source ทดสอบว่าต้องติดตั้ง ROS2 เรียบร้อยแล้ว

```bash
source /opt/ros/iron/setup.bash
ros2
```
หลังจากนั้น Update Ubuntu

```bash
sudo apt update
sudo apt upgrade
```

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

```bash
sudo apt update
sudo apt install snapd
sudo snap install code --classic
```

เมื่อติดตั้ง vscode เสร็จทำการเปิดโปรแกรม

```bash
code .
```

หลังจากเข้ามา vscode ทำการติดตั้ง Extensions ที่ publish by Microsoft
- ROS
- Python

ติดตั้ง python3-pip สำหรับการ build package

```bash
sudo apt install python3-pip
```
## Create First Package
เมื่อเราติดตั้ง ROS2 และ Tools ต่างๆจนครบ \
เราจะพร้อมที่จะสร้าง Package เพื่อใช้งานแล้วตอนนี้ \
เริ่มต้นด้วยการเปิด terminal ขึ้นมา ทำการสร้าง Folder ros2_ws สำหรับ ROS2 \ 

```bash
mkdir ros2_ws
colcon build

cd ros2_ws
mkdri src
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
