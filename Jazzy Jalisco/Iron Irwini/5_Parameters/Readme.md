
## Create Parameter

หากเราต้องการสร้างตัวแปรที่สามารถรับค่าได้ นิยมใช้งานส่วนของ Parameter ใน Ros2\ จะยังคงใช้งานส่วนของ msg ที่เคยสร้างไว้แล้ว\
เปิด terminal ทำการเข้าไปยัง Folder ce_robot ของ package ce_robot
```bash
cd ros2_ws/src/ce_robot/ce_robot
```

สร้าง file python ที่มีขื่อว่า HwStatus_para_publish.py

```bash
touch HwStatus_para_publish.py
chmod +x HwStatus_para_publish.py
```

เขียน code ด้วยภาษา python ใน file HwStatus_para_publish.py ตามตัวอย่าง\
เสร็จแล้วทำการเพื่อ file setup.py เพื่อให้สามารถ colcon build ได้ใน Ros2
เพิ่ม code ภายใต้ 'console_scripts': [ ] ของ file setup.py

- "hw_para = ce_robot.HwStatus_para_publish:main",
```bash
cd ~/ros2_ws
code .
```

เมื่อ Code ไม่มี error แล้วต้องทำการ Colcon build เพื่อให้ Package
ที่เราสร้างขึ้นสามารถใช้งานผ่าน คำสั่ง ros2 run ได้
```bash
cd ~/ros2_ws
colcon build --packages-select ce_robot --symlink-install
```

ทดสอบ Parameter ที่สร้างขึ้นใน Node publish สามารถใช้งานได้หรือไม่\
เปิดอีก terminal เข้า source ./bashrc แล้วใช้คำสั่ง ros2 run package ที่สร้างขึ้น
```bash
source ~/.bashrc
ros2 run ce_robot hw_para 
```

เมื่อ run node publish เสร็จแล้วทำการทดสอบ\
เปิดอีก terminal เข้า source ./bashrc แล้วใช้คำสั่ง ros2 param list
```bash
source ~/.bashrc
ros2 param list 
```

เมื่อใช้คำสั่ง list แล้วจะเห็น Parameter ที่สามารถแก้ไขได้ \
การแก้ไข Parameter ต้องทำการ ปิด Node HwStatus_para_publish ที่กำลังทำงานอยู่\
เสร็จแล้วทำการเปิด Node HwStatus_para_publish ขึ้นมาใหม่โดยใช้คำสั่ง ต่อไปนี้\
"rb-ce" คือชื่อที่แก้ใน Parameter rb_name\
1780 คือค่าที่แก้ใน Parameter rb_no\
```bash
ros2 run ce_robot hw_para --ros-args -p rb_name:="rb-ce" -p rb_no:=1789
```

Directory Tree
```bash
|--ros2_ws
   |--build
   |--intstall
   |--log
   |--src
      |--ce_robot_interfaces
      |  |--msg
      |  |  |--HardwareStatus.msg
      |  |--srv
      |     |--CalRectangle.srv
      |--ce_robot
         |--ce_robot
            |--first_node.py
            |--first_publisher.py
            |--first_subscriber.py
            |--add_two_ints_server.py
            |--add_two_ints_client.py
            |--HardwareStatus_publish.py
            |--CalRect_server.py
            |--CalRect_client.py
            |--HwStatus_para_publish.py    
```
