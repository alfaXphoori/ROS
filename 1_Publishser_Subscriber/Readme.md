
## Create Publisher / Subscriber Node
การสร้าง Node Publisher เป็นการสร้าง Node เพื่อส่งข้อมูลออกไป\
เปิด terminal ทำการเข้าไปยัง Folder ce_robot ของ package ce_robot  

```bash
cd ros2_ws/src/ce_robot/ce_robot
```

สร้าง file python ที่ชื่อว่า first_publisher.py
```bash
touch first_publisher.py
chmod +x first_publisher.py
```

จากนั้นทำการเขียน Code ภาษา python เมื่อเสร็จแล้วทำการทดสอบ file โดยใช้คำสั่ง 
```bash
./first_publisher.py
```

สร้าง Node Subscriber เป็นการสร้าง Node เพื่อรับข้อมูลจาก Publisher Node\
```bash
cd ros2_ws/src/ce_robot/ce_robot
```

สร้าง file python ที่ชื่อว่า first_subscriber.py
```bash
touch first_subscriber.py
chmod +x first_subscriber.py
```

จากนั้นทำการเขียน Code ภาษา python เมื่อเสร็จแล้วทำการทดสอบ file โดยใช้คำสั่ง 
```bash
./first_subscriber.py
```

แก้ไข file package.xml โดยเพิ่ม code ส่วน library\
เพิ่ม code ภายใต้ 'console_scripts': [ ] ของ file setup.py
- "first_pub = ce_robot.first_publisher:main",
- "first_sub = ce_robot.first_subscriber:main",

เมื่อ Code ไม่มี error แล้วต้องทำการ Colcon build เพื่อให้ Package \
ที่เราสร้างขึ้นสามารถใช้งานผ่าน คำสั่ง ros2 run ได้
```bash
cd ~/ros2_ws
colcon build --packages-select ce_robot --symlink-install
```

เมื่อทำการ colcon build สำเร็จ ทำการทดสอบการ ทำงาน Package ที่สร้างขึ้นโดย \
เปิด terminal เข้า source ./bashrc แล้วใช้คำสั่ง ros2 run package ที่สร้างขึ้น
```bash
source ~/.bashrc
ros2 run ce_robot first_pub
```

เปิดอีก terminal เข้า source ./bashrc แล้วใช้คำสั่ง ros2 run package ที่สร้างขึ้น
```bash
source ~/.bashrc
ros2 run ce_robot first_sub
```

เปิดอีก terminal เข้า source ./bashrc แล้วใช้คำสั่ง rqt_graph เพื่อดูการเชื่อมต่อของ Node
```bash
source ~/.bashrc
rqt_graph

Tree Directory
```bash
|--ros2_ws
   |--build
   |--intstall
   |--log
   |--src
      |--ce_robot
         |--ce_robot
            |--first_node.py
            |--first_publisher.py
            |--first_subscriber.py
          
```
```
