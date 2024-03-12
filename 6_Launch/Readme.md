## Create Launch
สำหรับ Node ใน Ros2 มีจำนวนมาก ในหลายครั้งเราต้องการใช้งาน Node นั้น พร้อมกัน\
การที่ต้องเช้า terminal แล้วไปเปิด Node นั้นจริงเป็นเรื่องยาก \
ดั้งนั้นเพื่อให้ง่ายต่อการใช้งานหลาย Node จึงจำเป็นต้องสร้าง Package Launch \
ขึ้นมาเพื่อช่วยในการ run Node หลายอย่างพร้อมกันได้ง่าย
เข้าไปยัง ros2_ws/src สำหรับสร้าง Package ที่มีชื่อว่า ce_robot_bootup
```bash
cd ros2_ws/src
ros2 pkg create ce_robot_bootup
```

เมื่อสร้าง Package เสร็จแล้วให้เข้าไปยัง ce_robot_bootup พร้อมลบ Directory ที่ไม่จำเป็นทิ้ง\
และทำการสร้าง Directory ใหม่ที่ชื่อว่า launch ขึ้นมา
```bash
rm -rf include
rm -rf src
mkdir launch
```

สร้าง file python ชื่อว่า ce_boot.launch.py สำหรับการเขียน Code สั่งาน\
พร้อมกับเปลี่ยน Permission ให้เป็น +x
 ```bash
cd launch
touch ce_boot_launch.py
chmod+x ce_robot_boot_launch.py
```

เมื่อเสร็จแล้วให้ทำการเขียน Code python ตามตัวอย่างที่มีไว้ให้แล้ว
 ```bash
cd ~/ros2_ws/src/ce_robot_bootup
code .
```

เมื่อเขียน Code เสร็จเรียบร้อยแล้วออกยังมายัง ros2_ws และทำการ colcon build 
 ```bash
cd ~/ros2_ws
colcon build --packages-select ce_robot_bootup --symlink-install
```

ทดสอบ launch package ที่สร้างขึ้นโดย\
เปิดอีก terminal เข้า source ./bashrc แล้วใช้คำสั่ง ros2 run launch\
เมื่อ launch แล้ว จะเป็นการเปิด run ทั้ง 3 Node พร้อมกันตาม Code ที่ขัยนไว้
```bash
source ~/.bashrc
ros2 launch ce_robot_bootup ce_boot_launch.py 
```

ทดสอบ node ที่รันว่า ros2 สามารถใช้งานได้หรือไม่โดยการ\
เปิดอีก terminal เข้า source ./bashrc แล้วใช้คำสั่ง ros2 node list\
จะได้ผลรับเป็น node ที่พร้อมใช้งานคือ CalRect_sv, HWStatus_para, HWStatus_pub
```bash
source ~/.bashrc
ros2 node list
```

ทดสอบ Node Publisher HWStatus_para, HWStatus_pub โดยการ echo ไปยัง topic\
ตรวจสอบ topic ทีทำงานอยู่ ด้วยคำสั่ง ros2 topic list
```bash
ros2 topic list
ros2 topic echo /hw_status
ros2 topic echo /hw_parameter
```

ทดสอบ Node Sever CalRect_sv โดยการ call ไปยัง service ที่เปิดไว้\
ตรวจสอบ service ทีทำงานอยู่ด้วยคำสั่ง ros2 service list
```bash
ros2 service list
ros2 service call /cal_rectangle ce_robot_interfaces/srv/CalRectangle {"length: 12.13, width: 4.9}"
```
