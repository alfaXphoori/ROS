# **Advanced Publisher & Subscriber Patterns in ROS 2**

## **📌 Project Title**

Multiple Publishers & Subscribers with Sensor Data Integration

## **👤 Authors**

- [@alfaXphoori](https://www.github.com/alfaXphoori)

## **🛠 Overview**

This advanced example demonstrates:
1. **Multiple Publishers** - Individual sensor nodes publishing different data types
2. **Multiple Subscribers** - Multiple subscriber nodes processing the same data differently
3. **Combined Publisher** - Aggregating data from multiple sources
4. **Data Logging** - Subscriber that logs all sensor data to a file
5. **Data Visualization** - Subscriber that displays real-time dashboard

---

## **📊 Architecture**

```
┌─────────────────────────────────────┐
│  Temperature Publisher              │
│  (sensor/temperature)               │
└────────────┬────────────────────────┘
             │
┌─────────────────────────────────────┐
│  Humidity Publisher                 │
│  (sensor/humidity)                  │
└────────────┬────────────────────────┘
             │
┌─────────────────────────────────────┐
│  Pressure Publisher                 │
│  (sensor/pressure)                  │
└────────────┬────────────────────────┘
             │
        ┌────▼────────────────┐
        │                     │
    ┌───▼──────────┐    ┌────▼──────────┐
    │ Logger       │    │ Visualizer    │
    │ Subscriber   │    │ Subscriber    │
    └──────────────┘    └───────────────┘
```

---

## **🚀 Running the Example**

### **Step 1: Navigate to the workspace**

```bash
cd ~/ros2_ws
```

### **Step 2: Build the package** (if needed)

```bash
colcon build --packages-select ce_robot --symlink-install
```

### **Step 3: Source the setup file**

```bash
source install/setup.bash
```

### **Step 4: Open Multiple Terminals**

#### **Terminal 1 - Temperature Publisher**
```bash
python3 Advanced_PubSub/temp_publisher.py
```

#### **Terminal 2 - Humidity Publisher**
```bash
python3 Advanced_PubSub/humidity_publisher.py
```

#### **Terminal 3 - Pressure Publisher**
```bash
python3 Advanced_PubSub/pressure_publisher.py
```

#### **Terminal 4 - Logger Subscriber** (logs to `/tmp/sensor_data.log`)
```bash
python3 Advanced_PubSub/logger_subscriber.py
```

#### **Terminal 5 - Visualizer Subscriber** (displays live dashboard)
```bash
python3 Advanced_PubSub/visualizer_subscriber.py
```

#### **Terminal 6 - Combined Sensor Publisher** (optional)
```bash
python3 Advanced_PubSub/sensor_combined_publisher.py
```

---

## **📁 Files Description**

### **Custom Message**
- **`SensorData.msg`** - Custom message type with temperature, humidity, and pressure fields

### **Publishers**
- **`temp_publisher.py`** - Publishes temperature data to `sensor/temperature` topic
- **`humidity_publisher.py`** - Publishes humidity data to `sensor/humidity` topic
- **`pressure_publisher.py`** - Publishes pressure data to `sensor/pressure` topic
- **`sensor_combined_publisher.py`** - Subscribes to all three topics and publishes combined data

### **Subscribers**
- **`logger_subscriber.py`** - Logs all sensor data to `/tmp/sensor_data.log`
- **`visualizer_subscriber.py`** - Displays real-time dashboard with sensor readings

---

## **🔍 Monitoring Topics**

### **View Available Topics**
```bash
ros2 topic list
```

### **View Topic Messages**
```bash
ros2 topic echo /sensor/temperature
ros2 topic echo /sensor/humidity
ros2 topic echo /sensor/pressure
```

### **View Topic Information**
```bash
ros2 topic info /sensor/temperature
```

### **View Node Graph**
```bash
rqt_graph
```

---

## **📊 Key Concepts**

### **1. Multiple Publishers**
- Each sensor publishes independently on its own topic
- Publishers run at different rates (all 1Hz in this example)
- Data is isolated and can be processed independently

### **2. Multiple Subscribers**
- Multiple nodes subscribe to the same topics
- Each subscriber processes data according to its purpose
- Logger subscriber writes to disk, visualizer creates dashboard
- Subscribers operate independently without blocking each other

### **3. Publisher-Subscriber Decoupling**
- Publishers don't know about subscribers
- Subscribers don't know about publishers
- Topics decouple the communication
- New subscribers can be added without modifying publishers

### **4. Data Aggregation**
- Combined publisher subscribes to all sensor topics
- Aggregates data from multiple sources
- Creates a single unified data source if needed

---

## **🛠 Customization Ideas**

### **1. Add More Sensor Types**
```bash
# Create new publishers for:
# - Humidity Publisher
# - CO2 Level Publisher
# - Light Intensity Publisher
```

### **2. Add Data Processing**
```python
# Subscriber that:
# - Filters data
# - Calculates averages
# - Detects anomalies
# - Triggers alerts
```

### **3. Add Database Logging**
```python
# Instead of file logging:
# - Store in SQLite
# - Use ROS 2 bag files
# - Send to cloud database
```

### **4. Add Web Dashboard**
```bash
# Display sensor data via:
# - Web server (Flask/FastAPI)
# - WebSocket for real-time updates
# - Grafana integration
```

### **5. Add Filtering & Smoothing**
```python
# Process sensor data with:
# - Moving average filters
# - Kalman filter
# - Low-pass filter
```

---

## **📂 Directory Structure**

```
📁 Advanced_PubSub/
├── 📄 SensorData.msg
├── 🐍 temp_publisher.py
├── 🐍 humidity_publisher.py
├── 🐍 pressure_publisher.py
├── 🐍 sensor_combined_publisher.py
├── 🐍 logger_subscriber.py
├── 🐍 visualizer_subscriber.py
└── 📄 Readme.md
```

---

## **🎯 Learning Outcomes**

After completing this example, you'll understand:
- ✅ How to create multiple independent publishers
- ✅ How to create multiple subscribers on the same topics
- ✅ Topic-based communication in ROS 2
- ✅ Data aggregation and processing
- ✅ Logging and visualization patterns
- ✅ Decoupled architecture design

---

## **🔗 Related Topics**

- Services & Clients (synchronous communication)
- Actions (long-running tasks)
- Custom Message Types
- QoS (Quality of Service) Settings
- Node Lifecycle Management
- ROS 2 Launch Files

---

**✅ Advanced Pub/Sub Example Complete!** 🚀✨
