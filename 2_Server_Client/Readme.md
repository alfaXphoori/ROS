## Create Server / Client Node

การสร้าง Node Server เป็นการสร้าง Node เพื่อส่งข้อมูลออกไป\
เปิด terminal ทำการเข้าไปยัง Folder ce_robot ของ package ce_robot  

```bash
cd ros2_ws/src/ce_robot/ce_robot
```

สร้าง file python ที่ชื่อว่า add_two_ints_server.py
```bash
touch add_two_ints_server.py
chmod +x add_two_ints_server.py
```

จากนั้นทำการเขียน Code ภาษา python เมื่อเสร็จแล้วทำการทดสอบ file โดยใช้คำสั่ง 
```bash
./add_two_ints_server.py
```

แก้ไข file package.xml โดยเพิ่ม code ส่วน library\
เพิ่ม code ภายใต้ 'console_scripts': [ ] ของ file setup.py
- "add_two_server = ce_robot.add_two_ints_server:main",

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
ros2 run ce_robot add_two_server
```

เปิดอีก terminal เข้า source ./bashrc แล้วใช้คำสั่ง ros2 service call เพื่อทดสอบ server\ แล้วจะได้ผลลัพธ์เป็นผล + ของตัวเลข 2 ตัว
```bash
source ~/.bashrc
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 4, b: 5}"
```

สร้าง Node Client เป็นการสร้าง Node เพื่อส่งค่าไปยัง Server Node\
```bash
cd ros2_ws/src/ce_robot/ce_robot
```

สร้าง file python ที่ชื่อว่า add_two_ints_client.py
```bash
touch add_two_ints_client.py
chmod +x add_two_ints_client.py
```

จากนั้นทำการเขียน Code ภาษา python เมื่อเสร็จแล้วทำการทดสอบ file โดยใช้คำสั่ง 
```bash
./add_two_ints_client.py
```

แก้ไข file package.xml โดยเพิ่ม code ส่วน library\
เพิ่ม code ภายใต้ 'console_scripts': [ ] ของ file setup.py
- "add_two_client = ce_robot.add_two_ints_client:main",

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
ros2 run ce_robot add_two_client 10 20 
```

เปิดอีก terminal เข้า source ./bashrc แล้วใช้คำสั่ง rqt_graph เพื่อดูการเชื่อมต่อของ Node
```bash
source ~/.bashrc
rqt_graph
```
