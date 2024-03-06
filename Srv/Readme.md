
## Create Srv
การสร้าง Mag เพื่อใช้เป็นตัวเแปลในการส่งข้อมูล
เปิด terminal ทำการเข้าไปยัง Folder ce_robot_interfaces
```bash
cd ros2_ws/src/ce_robot_interfaces
```

สร้าง Folder สำหรับเก็บ srv
```base
mkdir srv
```

เข้าไปยัง Folder msg และสร้าง file CalRectangle.srv เพื่อเก็บค่า service\
ทำการเขียนค่า ตัวแปร srv ที่ต้องการ
```base
cd srv
touch CalRectangle.srv
code .
```

ไปยัง CMakeLists แล้วทำการเพิ่มค่าตามนี้
- rosidl_generate_interfaces(${PROJECT_NAME}\
    "srv/CalRectangle.srv"\
  )

Build package ce_robot_interfaces เพื่อให้สามารถใช้งานได้
```base
cd ~/ros2_ws
colcon build --packages-select ce_robot_interfaces
```

ทดสอบ srv ที่สร้างขึ้นมาโดยใช้คำสั่ง ต่อไปนี้
```base
ros2 interface show ce_robot_interfaces/srv/CalRectangle 
```

สร้าง Node Server เพื่อส่งค่าผ่าน srv ที่เราสร้างขึ้น\
เปิด terminal ทำการเข้าไปยัง Folder ce_robot ของ package ce_robot  

```bash
cd ros2_ws/src/ce_robot/ce_robot
```

สร้าง file python ที่ชื่อว่า CalRectangle_Server.py
```bash
touch CalRectangle_Server.py
chmod +x CalRectangle_Server.py
```

จากนั้นทำการเขียน Code ภาษา python เมื่อเสร็จแล้วทำการทดสอบ file โดยใช้คำสั่ง 
```bash
./CalRectangle_Server.py
```

แก้ไข file package.xml โดยเพิ่ม code ส่วน library\
เพิ่ม code ภายใต้ 'console_scripts': [ ] ของ file setup.py
- "cal_rect_server = ce_robot.CalRectangle_Server:main",

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
ros2 run ce_robot cal_rect_server
```

เปิด terminal ใหม่ เข้า source ./bashrc แล้วใช้คำสั่ง ros2 echo เพื่อตรวจสอบตัวแปร msg
```bash
ros2 service call /cal_rect ce_robot_interfaces/srv/CalRectangle "{length: 55.20, width: 33.12}
```

สร้าง Node Client เพื่อส่งค่าผ่าน srv ที่เราสร้างขึ้น\
เปิด terminal ทำการเข้าไปยัง Folder ce_robot ของ package ce_robot  

```bash
cd ros2_ws/src/ce_robot/ce_robot
```

สร้าง file python ที่ชื่อว่า CalRectangle_Client.py
```bash
touch CalRectangle_Client.py
chmod +x CalRectangle_Client.py
```

จากนั้นทำการเขียน Code ภาษา python เมื่อเสร็จแล้วทำการทดสอบ file โดยใช้คำสั่ง 
```bash
./CalRectangle_Client.py
```

แก้ไข file package.xml โดยเพิ่ม code ส่วน library\
เพิ่ม code ภายใต้ 'console_scripts': [ ] ของ file setup.py
- "cal_rect_client = ce_robot.CalRectangle_Client:main",

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
ros2 run ce_robot CalRectangle_Client 22.22 33.34
```
