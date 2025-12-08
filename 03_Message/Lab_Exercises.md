# **🚀 Custom Messages Lab Exercises**

Master custom message creation and usage in ROS 2 through progressive hands-on exercises.

---

## **📌 Project Title**

Create and Use Custom Message Types in ROS 2

## **👤 Authors**

- [@alfaXphoori](https://www.github.com/alfaXphoori)

---

## **🛠 Lab Overview**

This lab provides hands-on exercises to master custom message creation and usage in ROS 2. Each exercise builds upon the previous one, progressing from basic message definition through production-quality validation and error handling.

**Duration:** ~90 minutes
**Level:** Beginner to Intermediate
**Prerequisites:** ROS 2 Jazzy installed, Publisher/Subscriber lab completed

---

## **🎯 Learning Objectives**

By completing this lab, you will be able to:

- ✅ Create custom message packages following ROS 2 conventions
- ✅ Define message types with appropriate fields and types
- ✅ Build and generate message interfaces with `colcon`
- ✅ Publish and subscribe to custom messages in Python
- ✅ Aggregate data from multiple messages for analysis
- ✅ Implement message validation and error handling
- ✅ Use logging levels appropriately (INFO, WARN, ERROR)
- ✅ Track reliability metrics and success rates
- ✅ Write defensive code that handles invalid data
- ✅ Debug and monitor custom message communication
---

## **📊 Lab Architecture**

```
┌─────────────────────────────────────────────┐
│ Exercise 1: Message Package Creation        │
│ (Create ce_robot_interfaces package)        │
└──────────────┬──────────────────────────────┘
               │
               ▼
┌─────────────────────────────────────────────┐
│ Exercise 2: Publisher Implementation        │
│ (Send HardwareStatus messages)              │
└──────────────┬──────────────────────────────┘
               │
               ▼
┌─────────────────────────────────────────────┐
│ Exercise 3: Data Aggregation                │
│ (Collect & analyze multiple messages)       │
└──────────────┬──────────────────────────────┘
               │
               ▼
┌─────────────────────────────────────────────┐
│ Exercise 4: Validation & Error Handling     │
│ (Production-quality implementation)         │
└─────────────────────────────────────────────┘
```

---

## **📚 Learning Path Overview**

| Exercise | Title | Level | Duration |
|----------|-------|-------|----------|
| 1 | Message Package Creation | Beginner | 20 min |
| 2 | Publisher Implementation | Beginner | 20 min |
| 3 | Data Aggregation | Intermediate | 25 min |
| 4 | Validation & Error Handling | Advanced | 25 min |

---

## **Exercise 1: Creating Custom Message Package 🔧**

Refer to **Readme.md** for complete instructions on creating the message package, defining HardwareStatus.msg, and configuring package.xml and CMakeLists.txt.

**Quick Summary:**
```bash
cd ~/ros2_ws/src
ros2 pkg create ce_robot_interfaces
cd ce_robot_interfaces
rm -rf include/ src/
mkdir msg
```

Define `msg/HardwareStatus.msg` with the following fields:
```msg
string name_robot          # Robot identifier/name
int64 number_robot         # Robot ID number (1-1000)
int64 temperature          # Temperature in Celsius (0-100)
bool motor_ready           # Motor operational status
string debug_message       # Status message for debugging
```

Then build and verify:
```bash
cd ~/ros2_ws
colcon build --packages-select ce_robot_interfaces
source install/setup.bash
ros2 interface show ce_robot_interfaces/msg/HardwareStatus
```

**See Readme.md for:**
- Detailed package.xml configuration
- Detailed CMakeLists.txt configuration
- Complete setup instructions with expected output

### ✅ Completion Criteria
- [ ] Created `ce_robot_interfaces` package
- [ ] Defined `HardwareStatus.msg` with 5 fields
- [ ] Built package successfully
- [ ] Verified message structure with `ros2 interface show`

---

## Exercise 2: Publisher with Custom Messages 📡

### 🎯 Objective
Create a publisher node that sends `HardwareStatus` messages with realistic robot data.

### 📝 Task

**Complete implementation reference:** See **Readme.md** for the full `HardwareStatus_publish.py` code with:
- CE-ROBOT name with robot_number 1001
- Random temperature (35-60°C)
- Timer callback publishing at 2 Hz
- Properly configured entry point: `03_hw_status_publisher`

**Quick Summary:**

1. Create `HardwareStatus_publish.py` in `ce_robot/src/` that:
   - Uses timer callback for 2 Hz publication (0.5 second interval)
   - Sets name_robot to 'CE-ROBOT'
   - Sets number_robot to 1001
   - Generates random temperature between 35-60°C
   - Sets motor_ready to True/False
   - Publishes to 'hardware_status' topic

2. Update `setup.py` entry point:
   ```python
   'entry_points': {
       'console_scripts': [
           '03_hw_status_publisher = ce_robot.HardwareStatus_publish:main',
       ],
   },
   ```

3. Build and test:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select ce_robot --symlink-install
   source install/setup.bash
   ros2 run ce_robot 03_hw_status_publisher
   ```

### 💡 Key Learning Points
- Import and instantiate custom messages
- Use timer callbacks for periodic publication
- Populate all message fields with appropriate data types
- ROS 2 enforces type safety at publish time
- Entry point naming should reflect functionality (descriptive names)

### ✅ Completion Criteria
- [ ] Created `HardwareStatus_publish.py` in `ce_robot/src/`
- [ ] Publisher runs at correct frequency (2 Hz)
- [ ] All 5 message fields populated
- [ ] Entry point configured in setup.py
- [ ] Build completes without errors
- [ ] Publisher successfully sends messages on 'hardware_status' topic

---

## Exercise 3: Multi-Field Message Aggregation 🔄

### 🎯 Objective
Create subscriber that collects and analyzes `HardwareStatus` messages, aggregating multiple field values.

### 📝 Task

**Complete implementation reference:** See **Readme.md** for the full `HardwareStatus_aggregate.py` code with:
- Deque-based history tracking (last 10 messages)
- Statistics calculation (average, min, max temperature)
- 5-second reporting interval
- Motor status aggregation
- Properly configured entry point: `03_hw_status_aggregator`

**Quick Summary:**

1. Create `HardwareStatus_aggregate.py` in `ce_robot/src/` that:
   - Subscribes to 'hardware_status' topic
   - Uses deque(maxlen=10) to track temperature history
   - Calculates: average, min, max temperature
   - Tracks motor status changes
   - Prints statistics every 5 seconds

2. Update `setup.py` entry point:
   ```python
   '03_hw_status_aggregator = ce_robot.HardwareStatus_aggregate:main',
   ```

3. Build and run with publisher:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select ce_robot --symlink-install
   source install/setup.bash

   # Terminal 1: Publisher
   ros2 run ce_robot 03_hw_status_publisher

   # Terminal 2: Aggregator
   ros2 run ce_robot 03_hw_status_aggregator
   ```

### 💡 Key Learning Points
- Message subscription with callback processing
- Data structure selection (deque for efficient rolling window)
- Statistical analysis of message streams
- Timer-based periodic reporting
- Tracking multiple sensor readings over time
- Aggregation patterns for data analysis

### ✅ Completion Criteria
- [ ] Created `HardwareStatus_aggregate.py` in `ce_robot/src/`
- [ ] Subscriber receives all published messages
- [ ] Temperature history stored in deque (max 10 items)
- [ ] Statistics calculated: average, min, max
- [ ] Motor status tracked across messages
- [ ] Statistics printed every 5 seconds
- [ ] Publisher and aggregator run simultaneously
- [ ] Statistics display shows correct calculations

---

## Exercise 4: Message Validation & Error Handling ⚠️

### 🎯 Objective
Implement robust message validation and error handling for production-quality code.

### 📝 Task

**Complete implementation reference:** See **Readme.md** for the full `HardwareStatus_validated.py` code with:
- Input validation for all message fields
- Range checking (temperature -40 to 100°C, robot_number 1-1000)
- Exception handling in publisher and subscriber
- Success metrics and reliability tracking
- Properly configured entry points: `03_hw_status_validated`, `03_hw_status_validated_sub`

**Quick Summary:**

1. Create `HardwareStatus_validated.py` in `ce_robot/src/` that:
   - Validates name_robot is not empty
   - Validates number_robot is 1-1000
   - Validates temperature is -40 to 100°C
   - Validates debug_message ≤ 200 characters
   - Tracks error count and success rate
   - Reports statistics every 10 cycles

2. Create `HardwareStatus_validated_sub.py` that:
   - Handles missing message fields gracefully
   - Alerts on high temperature (>75°C)
   - Alerts on motor offline status
   - Counts received and error messages
   - No crashes on malformed data

3. Update `setup.py` entry points:
   ```python
   '03_hw_status_validated = ce_robot.HardwareStatus_validated:main',
   '03_hw_status_validated_sub = ce_robot.HardwareStatus_validated_sub:main',
   ```

4. Build and run with validation:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select ce_robot --symlink-install
   source install/setup.bash

   # Terminal 1: Validated publisher
   ros2 run ce_robot 03_hw_status_validated

   # Terminal 2: Validated subscriber
   ros2 run ce_robot 03_hw_status_validated_sub
   ```

### 💡 Key Learning Points
- Input validation prevents invalid data from propagating
- Try-catch blocks make code resilient to errors
- Logging levels (INFO, WARN, ERROR) indicate severity
- Defensive programming assumes data might be invalid
- Tracking success/failure metrics enables reliability monitoring
- AlertCallback mechanisms notify operators of problems
- Production code requires comprehensive error handling

### ✅ Completion Criteria
- [ ] Created `HardwareStatus_validated.py` with validation logic
- [ ] Implemented range checks for all message fields
- [ ] Added error handling in publisher and subscriber
- [ ] Added temperature alert (>75°C) and motor status alert
- [ ] Entry points configured in setup.py
- [ ] Build completes without errors
- [ ] Both publisher and subscriber run without crashes
- [ ] Success rate statistics calculated and logged
- [ ] Validation correctly identifies out-of-range values

---

## 🔑 Key Concepts Summary

### Message Architecture
- **Message Definition** (`.msg` files): Declarative schema for data structures
- **Package Organization**: Custom messages in dedicated `*_interfaces` packages
- **Type System**: Primitive types (int, float, bool, string) and collections
- **Field Naming**: snake_case for clarity and consistency

### Communication Patterns
- **One-to-Many**: Single publisher, multiple subscribers
- **Many-to-One**: Multiple publishers, single subscriber (aggregation)
- **Publish-Subscribe**: Asynchronous, decoupled communication
- **Quality of Service (QoS)**: Message queue depth, reliability settings

### Best Practices
- **Validation**: Always validate received messages
- **Error Handling**: Use try-catch and logging for resilience
- **Performance**: Use appropriate field types to minimize message size
- **Documentation**: Include units and constraints in field comments
- **Testing**: Test with edge cases and invalid data

---

## 🛠️ Troubleshooting Guide

### Issue: "Cannot find module ce_robot_interfaces"
**Cause**: Message package not built or not sourced  
**Solution**:
```bash
colcon build --packages-select ce_robot_interfaces
source ~/ros2_ws/install/setup.bash
```

### Issue: "CMake Error: rosidl_generate_interfaces not found"
**Cause**: Missing rosidl build dependency  
**Solution**: Ensure `package.xml` has `<build_depend>rosidl_default_generators</build_depend>`

### Issue: Publisher message shows as different type
**Cause**: Old build artifacts not cleared  
**Solution**:
```bash
cd ~/ros2_ws
rm -rf build install log
colcon build --packages-select ce_robot_interfaces ce_robot
```

### Issue: Subscriber doesn't receive messages
**Cause**: Topic names don't match or QoS incompatible  
**Solution**:
```bash
# Check active topics
ros2 topic list
# Check message type
ros2 topic info /hardware_status
# Echo messages to verify
ros2 topic echo /hardware_status
```

---

## 📚 Additional Resources

### Official ROS 2 Documentation
- [Creating Custom Interfaces](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html)
- [Interfaces (Messages, Services, Actions)](https://docs.ros.org/en/jazzy/Concepts/About-ROS-Interfaces.html)
- [ROS 2 Message Type Reference](https://docs.ros.org/en/jazzy/Concepts/About-ROS-Interfaces.html#builtin-types)

### ROS 2 CLI Commands
```bash
# List all message types
ros2 interface list

# Show message structure
ros2 interface show package_name/msg/MessageName

# Find message definitions
find ~/ros2_ws -name "*.msg"

# Check topic information
ros2 topic info /topic_name
ros2 topic type /topic_name
```

### Message Type Best Practices
- Use appropriate types: `int32` vs `int64` vs `float32` vs `float64`
- Include field documentation with units: `int32 temperature_celsius`
- Avoid deeply nested messages for beginner projects
- Use consistent naming: `name_robot` not `robot_name`
- Consider versioning comments for future compatibility

---

## ✅ Lab Completion Checklist

### Exercise 1: Message Package
- [ ] Created `ce_robot_interfaces` package
- [ ] Defined `HardwareStatus.msg` with 5 fields
- [ ] Updated `package.xml` and `CMakeLists.txt`
- [ ] Built package without errors
- [ ] Verified message with `ros2 interface show`

### Exercise 2: Publisher
- [ ] Created `HardwareStatus_publish.py`
- [ ] Updated package dependencies
- [ ] Built and ran publisher successfully
- [ ] Messages visible with `ros2 topic echo`
- [ ] All fields populated correctly

### Exercise 3: Aggregation
- [ ] Created subscriber with aggregation logic
- [ ] Implemented deque-based history tracking
- [ ] Calculated statistics (avg, min, max)
- [ ] Publisher and subscriber run together
- [ ] Statistics output displays correctly

### Exercise 4: Validation
- [ ] Created validated publisher with checks
- [ ] Implemented error handling in publisher
- [ ] Created validated subscriber with error handling
- [ ] Added temperature and motor status alerts
- [ ] Tracked and reported success metrics
- [ ] No crashes on invalid data

### Overall Completion
- [ ] All 4 exercises completed
- [ ] All Python files created and tested
- [ ] Custom message package working
- [ ] Publisher/subscriber communication verified
- [ ] Error handling implemented
- [ ] Statistics and validation working

---

**🎓 Congratulations!** You've mastered custom message creation and usage in ROS 2! 🚀✨
