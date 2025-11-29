## 🚀 Create Custom Message (Msg) in ROS 2

Setting up a **custom message** allows communication between nodes using user-defined data structures. This guide walks you through creating and using the `HardwareStatus` custom message type.

### 📚 Learning Objectives

- Understand ROS 2 message definition syntax
- Create a custom message package with proper configuration
- Use custom messages in publisher/subscriber applications
- Build and verify custom interfaces using `colcon`
- Integrate custom messages into ROS 2 packages

---

## 📦 Creating a Custom Msg Package

### 🛠️ Creating the `ce_robot_interfaces` Package

Navigate to the `src` folder and create a new package:

```bash
cd ~/ros2_ws/src
ros2 pkg create ce_robot_interfaces
```

Remove unnecessary folders:

```bash
cd ce_robot_interfaces
rm -rf include/
rm -rf src
```

Create a folder for storing message definitions:

```bash
mkdir msg
code .
```

---

### 📌 Updating `package.xml` & `CMakeLists.txt`

Modify `package.xml` by adding the following lines under `<buildtool_depend>`:

```xml
<build_depend>rosidl_default_generators</build_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

Modify `CMakeLists.txt` by adding:

```cmake
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/HardwareStatus.msg"
)
```

---

## 📝 Defining the Custom Message

Navigate to the `msg` folder and create a new message file:

```bash
cd msg
touch HardwareStatus.msg
```

Define message variables inside `HardwareStatus.msg`, such as:

```msg
uint8 temperature      # Temperature in Celsius (0-255)
uint8 humidity         # Humidity percentage (0-100)
uint8 battery_level    # Battery percentage (0-100)
bool motor_status      # True if motor is running
uint32 uptime          # Uptime in seconds
```

**Key Points:**
- Message files use the `.msg` extension
- Each line defines a data member with type and name
- Comments explain field purpose
- Common types: `uint8`, `uint16`, `uint32`, `int32`, `float32`, `float64`, `string`, `bool`
- Arrays: use `type[size]` syntax (e.g., `uint8[3]` for 3-byte array)

---

### 🔨 Building the Package with Colcon

Compile the custom message package:

```bash
cd ~/ros2_ws
colcon build --packages-select ce_robot_interfaces
```

Verify the custom message structure:

```bash
ros2 interface show ce_robot_interfaces/msg/HardwareStatus
```

---

## 🚀 Using Custom Message in a Publisher

### 📡 Creating the Publisher Node

Navigate to the `ce_robot` package folder:

```bash
cd ~/ros2_ws/src/ce_robot/ce_robot
```

Create a Python file for the publisher:

```bash
touch HardwareStatus_publish.py
chmod +x HardwareStatus_publish.py
```

Write the necessary Python code and test the file using:

```bash
./HardwareStatus_publish.py
```

---

### 📌 Updating `package.xml` & `setup.py`

Modify `package.xml` to include necessary dependencies:

```xml
<build_depend>rosidl_default_generators</build_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<depend>ce_robot_interfaces</depend>
```

Update `setup.py` by adding the following under `console_scripts`:

```python
entry_points={
    'console_scripts': [
        "hw_status = ce_robot.HardwareStatus_publish:main",
    ],
},
```

Also ensure `ce_robot_interfaces` is added to `install_requires`:

```python
install_requires=['setuptools', 'ce_robot_interfaces'],
```

---

### 🔨 Building the Package with Colcon

Compile the package:

```bash
cd ~/ros2_ws
colcon build --packages-select ce_robot --symlink-install
```

---

### 🚀 Running and Testing the Publisher

Open a terminal and run the **Publisher**:

```bash
source ~/.bashrc
ros2 run ce_robot hw_status
```

Open a new terminal and echo the topic to verify message transmission:

```bash
ros2 topic echo /hardware_status
```

---

### 🗂️ Directory Structure

```bash
|--ros2_ws
   |--build
   |--install
   |--log
   |--src
      |--ce_robot_interfaces
         |--msg
            |--HardwareStatus.msg
      |--ce_robot
         |--ce_robot
            |--HardwareStatus_publish.py
```

✅ **Setup Complete!** 🚀✨

---

## 🔑 Key Concepts

### Message Packages
- Custom messages must be in dedicated packages (not application packages)
- Naming convention: `*_interfaces` (e.g., `ce_robot_interfaces`)
- Provides clear separation between message definitions and implementations

### Type System
| Type | Size | Range | Example |
|------|------|-------|---------|
| `uint8` | 1 byte | 0-255 | Battery level (%) |
| `uint16` | 2 bytes | 0-65,535 | Sensor reading |
| `int32` | 4 bytes | -2.1B to 2.1B | Temperature (×100) |
| `float32` | 4 bytes | IEEE 754 | Precise measurements |
| `string` | Variable | Any text | Device name |
| `bool` | 1 byte | true/false | Status flags |

### Build Process
1. **Message Definition** → Define `.msg` files with field types
2. **CMake Configuration** → Specify messages to generate interfaces
3. **Code Generation** → `colcon build` auto-generates Python/C++ classes
4. **Integration** → Import generated message classes in publishers/subscribers

---

## 🛠️ Common Troubleshooting

### Issue: "Cannot find module ce_robot_interfaces"
**Solution:**
1. Ensure `ce_robot_interfaces` package was built: `colcon build --packages-select ce_robot_interfaces`
2. Source the setup file: `source ~/ros2_ws/install/setup.bash`
3. Verify message generated: `ros2 interface show ce_robot_interfaces/msg/HardwareStatus`

### Issue: "colcon build" fails with CMakeLists.txt error
**Solution:**
1. Check package name in `CMakeLists.txt` matches `package.xml`
2. Verify message file path is correct: `msg/HardwareStatus.msg`
3. Ensure all dependencies listed in `package.xml` are installed

### Issue: "No module named 'ce_robot_interfaces'" at runtime
**Solution:**
1. Check if package is built: `ls ~/ros2_ws/install/ce_robot_interfaces/`
2. Source install directory: `source ~/ros2_ws/install/setup.bash`
3. Add dependency in consumer package's `setup.py`

### Issue: Message fields not recognized in publisher
**Solution:**
1. Rebuild consumer package: `colcon build --packages-select ce_robot --symlink-install`
2. Verify message structure: `ros2 interface show ce_robot_interfaces/msg/HardwareStatus`
3. Check import statement: `from ce_robot_interfaces.msg import HardwareStatus`

---

## 📚 Additional Resources

### Official ROS 2 Documentation
- [Creating Custom Messages](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html)
- [Interface Definition Language (IDL)](https://docs.ros.org/en/jazzy/Concepts/About-ROS-Interfaces.html)
- [Message Type Reference](https://docs.ros.org/en/jazzy/Concepts/About-ROS-Interfaces.html#builtin-types)

### ROS 2 Command Reference
```bash
# List all available messages
ros2 interface list

# Show specific message structure
ros2 interface show package_name/msg/MessageName

# Find message files in filesystem
find ~/ros2_ws -name "*.msg"

# Check message generation status
colcon build --packages-select package_name --cmake-args -DCMAKE_BUILD_TYPE=Debug
```

### Message Type Best Practices
- Use appropriate data types to minimize message size
- Include units in field comments (e.g., "temperature in Celsius")
- Avoid nesting messages for beginners (advanced feature)
- Use descriptive field names (avoid abbreviations when possible)
- Add version comments for future compatibility: `# v1.0`

---

## ✅ Completion Checklist

- [ ] Created `ce_robot_interfaces` package
- [ ] Defined `HardwareStatus.msg` with appropriate fields
- [ ] Updated `package.xml` with build dependencies
- [ ] Updated `CMakeLists.txt` with `rosidl_generate_interfaces()`
- [ ] Built interfaces: `colcon build --packages-select ce_robot_interfaces`
- [ ] Verified message structure: `ros2 interface show ce_robot_interfaces/msg/HardwareStatus`
- [ ] Created publisher node: `HardwareStatus_publish.py`
- [ ] Updated consumer package's `package.xml` with dependency
- [ ] Updated consumer package's `setup.py` with dependency and entry point
- [ ] Built consumer package: `colcon build --packages-select ce_robot`
- [ ] Ran publisher: `ros2 run ce_robot hw_status`
- [ ] Verified message in terminal: `ros2 topic echo /hardware_status`


