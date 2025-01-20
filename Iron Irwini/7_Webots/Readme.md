
## Install Webots
Installation
Install the webots, webots_ros2 package and run simulation examples on Ubuntu.
Start by importing the `Cyberbotics.asc`` signature file using these commands:
```bash
sudo mkdir -p /etc/apt/keyrings
cd /etc/apt/keyrings
sudo wget -q https://cyberbotics.com/Cyberbotics.asc
```
Adding the Cyberbotics repository:
```bash
echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/Cyberbotics.asc] https://cyberbotics.com/debian binary-amd64/" | sudo tee /etc/apt/sources.list.d/Cyberbotics.list
sudo apt update
```
Then proceed to the installation of Webots using:
```bash
sudo apt install webots
```
Add the installed version:
```bash
echo "export WEBOTS_HOME=/usr/local/webots" >> ~/.bashrc
source ~/.bashrc
```

Create a ROS 2 workspace with its src directory.
```bash
mkdir -p ~/ros2_ws/src
```
Source the ROS 2 environment.
```bash
source ~/.bashrc
```
Retrieve the sources from Github.
```bash
cd ~/ros2_ws
git clone --recurse-submodules https://github.com/cyberbotics/webots_ros2.git src/webots_ros2
```
Install the package dependencies.
```bash
sudo apt install python3-pip python3-rosdep python3-colcon-common-extensions
sudo rosdep init && rosdep update
rosdep install --from-paths src --ignore-src --rosdistro iron
```
Build the package using colcon.
```bash
cd ~/ros2_ws
colcon build
```

Run simulation Open teminal
```bash
ros2 launch webots_ros2_epuck robot_launch.py
```

Run teleop_twist_keyboard to Control
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
