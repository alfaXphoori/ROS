# Project Title

ROS2 Ubuntu 22_04

We chose Ros2 Iron Irwin to use.
## Authors

- [@alfaXphoori](https://www.github.com/alfaXphoori)


## Environment Variables

- VMware Workstation Pro
- Install Ubuntu 22.04.3 LTS (Jammy Jellyfish)


## Installation
 
Ros2 used the UTF-8 locale.

Use the command in Terminal. 

```bash
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify setting
```

We will need add the ROS2 apt respository to ubuntu.

```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
```

Now add the Ros2 GPG key with apy.
```bash
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```

Then add the repository to your sources list.
```bash 
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

Now install the development tools.
```bash
sudo apt update && sudo apt install ros-dev-tools
```

Install Ros2

Now update and upgrade Ubuntu.
```bash
sudo apt update
sudo apt upgrade
```

ROS-Desktop Install
```bash
sudo apt install ros-iron-desktop
```

ROS-Base Install
```bash
sudo apt install ros-iron-ros-base
```

Congratulations on the finished installation! 
## Usage/Examples

Now we need to test Ros2 on Ubuntu.\
First, open terminal on Ubuntu and set up the environment by sourcing.
```bash
source /opt/ros/iron/setup.bash
```

Second, check Ros2 on the environment.
```bash
printenv | grep -i ROS
```

Tye Examples\
We develop Ros2 using Python 3.\
In one terminal, choose source file and run.
```bash
source /opt/ros/iron/setup.bash
ros2 run demo_nodes_py talker
```

In another terminal, choose source file and run.
```bash
source /opt/ros/iron/setup.bash
ros2 run demo_nodes_py listener
```

We use rqt_graph to see the connection between the nodes.
```bash
source /opt/ros/iron/setup.bash
rqt_graph
```
Congratulations on the first time on ROS2.

Next we need to test Turtlesim.\
Turtlesim Install
```bash
sudo apt update
sudo apt install ros-iron-turtlesim 
ros2 pkg executables turtlesim
```

Run the test, Turtlesim.
In one terminal, choose source file and run.
```bash
ros2 run turtlesim turtlesim_node
```

In anoter terminal, choose source file and run.
```bash
source /opt/ros/iron/setup.bash
ros2 run turtlesim turtle_teleop_key
```

Now we can control turtles in Turtlesim by arrow on the keyboard.
## Create Package
If we need to create a package on Ros2,

First, we need to install Colcon.\
Open a terminal, choose source file, and check that ros2 is installed.

```bash
source /opt/ros/iron/setup.bash
ros2
```

Congratulations! We have Ros2 installed.  

Next, we need to update and upgrade Ubuntu.

```bash
sudo apt update
sudo apt upgrade
```

Colcon install and Check Colcon installed in the folder.
```bash
sudo apt install python3-colcon-common-extensions
cd /usr/share/colcon_argcomplete/hook/
```
Now, we need to add source locate to the last text on the bashrc file.

- source /opt/ros/iron/setup.bash
- source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash

```bash
gedit ~/.bashrc
```

Congratulations! We already have everything for creating packages on Ros2.

If we need to have a package on Ros2, \
we need to create a directory named ros2_ws and a subdirectory named src.
```bash
mkdir ros2_ws
cd ros2_ws
mkdir src
```
Using Colocon to build.
```bash
colcon build
```
Now, we need to add source locate to the last text on the bashrc file.
- source ~/ros2_ws/install/setup.bash
 ```bash
gedit ~/.bashrc
```
we go to src directory.
