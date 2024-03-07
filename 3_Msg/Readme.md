

## Create Msg
การสร้าง Mag เพื่อใช้เป็นตัวเแปลในการส่งข้อมูล
เปิด terminal ทำการเข้าไปยัง Folder src 
```bash
cd ros2_ws/src
```

สร้าง package ใหม่ที่มีชื่อว่า ce_robot_interfaces เพื่อใช้ในการเก็บค่า ตัวแปร
```bash
ros2 pkg create ce_robot_interfaces
```

เมื่อสร้าง package ทำการลบ folder ที่ไม่ต้องการทิ้ง
```bash
cd ce_robot_interfaces
rm -rf include/
rm -rf src
```

สร้าง Folder สำหรับเก็บ Msg 
```base
mkdir msg
code .
```

เพิ่ม 3 คำสั่ง ด้านล่างใน file package.xml ใต้ <buildtool_depend>ament_cmake</buildtool_depend>

- <build_depend>rosidl_default_generators</build_depend>
- <exec_depend>rosidl_default_runtime</exec_depend>
- <member_of_group>rosidl_interface_packages</member_of_group> 

พร้อมทำการเพิ่มคำสั่งใน CMakeLists.txt

- find_package(rosidl_default_generators REQUIRED)

- rosidl_generate_interfaces(${PROJECT_NAME}\
  "msg/HardwareStatus.msg"\
  )

เข้าไปยัง Folder msg และสร้าง file HardwareStatus.msg เพื่อสร้างตัวแปร
```base
cd msg
touch HardwareStatus.msg
```
เพิ่มตัวแปร int bool string ใน file HardwareStatus.msg ตามต้องการ

Build package ce_robot_interfaces เพื่อให้สามารถใช้งานได้
```base
cd ~/ros2_ws
colcon build --packages-select ce_robot_interfaces
```

ทดสอบ msg ที่สร้างขึ้นมาโดยใช้คำสั่ง ต่อไปนี้
```base
ros2 interface show ce_robot_interfaces/msg/HardwareStatus
```

สร้าง Node Publisher เพื่อส่งข้อผ่าน msg ที่เราสร้างขึ้น\
เปิด terminal ทำการเข้าไปยัง Folder ce_robot ของ package ce_robot  

```bash
cd ros2_ws/src/ce_robot/ce_robot
```

สร้าง file python ที่ชื่อว่า HardwareStatus_publish.py
```bash
touch HardwareStatus_publish.py
chmod +x HardwareStatus_publish.py
```

จากนั้นทำการเขียน Code ภาษา python เมื่อเสร็จแล้วทำการทดสอบ file โดยใช้คำสั่ง 
```bash
./HardwareStatus_publish.py
```

แก้ไข file package.xml โดยเพิ่ม code ส่วน library\
เพิ่ม code ภายใต้ 'console_scripts': [ ] ของ file setup.py
- "hw_status = ce_robot.HardwareStatus_publish:main",

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
ros2 run ce_robot HardwareStatus_publish
```

เปิด terminal ใหม่ เข้า source ./bashrc แล้วใช้คำสั่ง ros2 echo เพื่อตรวจสอบตัวแปร msg
```bash
ros2 topic echo /hardware_status
```
Tree Directory
```bash
|--ros2_ws
   |--build
   |--intstall
   |--log
   |--src
      |--ce_robot_interfaces
         |--msg
            |--HardwareStatus.msg
      |--ce_robot
         |--ce_robot
            |--first_node.py
            |--first_publisher.py
            |--first_subscriber.py
            |--add_two_ints_server.py
            |--add_two_ints_client.py
            |--HardwareStatus_publish.py
          
```
