# ros_install

ROS Indigo Ubuntu 14.04 Installation Step All in One

## ROS BASE

### 1. Install ROS & Gazebo

#### 1.1 Update & Upgrade Ubuntu Installation

```bash
sudo apt-get update
```

```bash
sudo apt-get upgrade
```

#### 1.2 Add ROS Source

```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu trusty main" > /etc/apt/sources.list.d/ros-latest.list'
```

#### 1.3 Add ROS Keys

```bash
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
```

#### 1.4 Update Repository List

```bash
sudo apt-get update
```

#### 1.5 Install ROS Desktop Full

```bash
sudo apt-get install ros-indigo-desktop-full
```

### 2. Setup ROS

#### 2.1 Initial ROSDEP Setup

```bash
sudo rosdep init
```

#### 2.2 Update ROSDEP

```bash
rosdep update
```

#### 2.3 Source BASH

```bash
echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
```

```bash
source ~/.bashrc
```

#### Install python-rosinstall

```bash
sudo apt-get install python-rosinstall
```

### 3. Setup CATKIN Workspace

#### 3.1 Create CATKIN Workspace & Source Folder

```bash
mkdir -p ~/catkin_ws/src
```

#### 3.2 Enter CATKIN Source Folder

```bash
cd ~/catkin_ws/src
```

#### 3.3 Initilize CATKIN Workspace

```bash
catkin_init_workspace
```

#### 3.4 Initial Build CATKIN Workspace

```bash
cd ~/catkin_ws
```

```bash
catkin_make
```

#### 3.5 Source BASH

```bash
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```

```bash
source ~/.bashrc
```

## AR.Drone

### 1. Clone Library & Install Library

#### 1.1 Enter CATKIN Source Folder

```bash
cd ~/catkin_ws/src
```

#### 1.2 Clone ROS Indigo Compatible Joystick Interface

```bash
git clone https://github.com/occominc/ardrone_joystick.git
```

#### 1.3 ROS Indigo Compatible Launch Scripts

```bash
git clone https://github.com/occominc/autonavx_ardrone.git
```

#### 1.4 ROS Indigo Compatible tum_simulator

```bash
git clone https://github.com/occominc/tum_simulator.git
```

#### 1.5 ROS Driver for Parrot [AR-Drone](http://wiki.ros.org/ardrone_autonomy) 1.0 and 2.0 quadrocopters

```bash
git clone https://github.com/AutonomyLab/ardrone_autonomy.git -b indigo-devel
```

#### 1.6 Wrappers, Tools and Additional [API's](http://wiki.ros.org/gazebo_ros_pkgs) for using ROS with Gazebo

```bash
git clone https://github.com/ros-simulation/gazebo_ros_pkgs.git -b indigo-devel
```

#### 1.7 TUM Ardrone for ArDrone GUI Control

```bash
git clone https://github.com/tum-vision/tum_ardrone -b indigo-devel
```

### 2. Install Dependencies

```bash
sudo apt-get install ros-indigo-gazebo-ros-pkgs ros-indigo-gazebo-ros-control
```

#### 2.1 Update ROSDEP

```bash
rosdep update
```

#### 2.2 Check Dependencies of ROS Packages

```bash
rosdep check --from-paths . --ignore-src --rosdistro indigo
```

#### 2.3 Install Dependencies of ROS Packages

```bash
rosdep install --from-paths . --ignore-src --rosdistro indigo -y
```

#### 2.4 Build CATKIN Packages

```bash
cd ~/catkin_ws
```

```bash
catkin_make
```

#### 2.5 Test Installation

Test simulation [Simulation](https://github.com/yanottamao/ardrone_gazebo_simulation) [Real](https://github.com/yanottamao/SKRIPSI)

## Troubleshooting

### 1. Update Couldn't Complete

#### 1.1 Delete Old Keys

```bash
sudo apt-key del B01FA116
```

#### 1.2 Add New Keys

```bash
sudo -E apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
```

### 2. ROS Dependency Issues

Issues [ROS Thread](https://answers.ros.org/question/203610/ubuntu-14042-unmet-dependencies-similar-for-14043/), [Launchpad](https://bugs.launchpad.net/ubuntu/trusty/+source/mesa/+bug/1424059)

#### 2.1 Ubuntu 14.04.2 Only

```bash
sudo apt-get install xserver-xorg-dev-lts-trusty mesa-common-dev-lts-trusty libxatracker-dev-lts-trusty libopenvg1-mesa-dev-lts-trusty libgles2-mesa-dev-lts-trusty libgles1-mesa-dev-lts-trusty libgl1-mesa-dev-lts-trusty libgbm-dev-lts-trusty libegl1-mesa-dev-lts-trusty
```

#### 2.2 Ubuntu 14.04 Only

```bash
sudo apt-get install libgl1-mesa-dev-lts-trusty
```

### 3. Gazebo Models

Download models from [Repository](bitbucket.org/osrf/gazebo_models/downloads/) then extract to:

```bash
/home/.gazebo/models/
```

## Other ROS Version

### 1. Desktop Full Install

Packages: [ROS](http://wiki.ros.org/ros), [RQT](http://wiki.ros.org/rqt), [RVIZ](http://wiki.ros.org/rviz), Gazebo 2, Robot-generic Libraries, 2D/3D Simulators, and 2D/3D Perception.

```bash
sudo apt-get install ros-indigo-desktop-full
```

### 2. Desktop Install

Packages: [ROS](http://wiki.ros.org/ros), [RQT](http://wiki.ros.org/rqt), [RVIZ](http://wiki.ros.org/rviz), Robot-generic Libraries.

```bash
sudo apt-get install ros-indigo-desktop
```

### 3. ROS-Base / Bare Bones

Packages: [ROS](http://wiki.ros.org/ros), Build, and Communication Libraries. No GUI Tools.

```bash
sudo apt-get install ros-indigo-ros-base
```

### 4. Individual ROS Package

Packages: [ROS](http://wiki.ros.org/ros), Specific ROS Package.

```bash
sudo apt-get install ros-indigo-PACKAGE_NAME
```

To Find Available Packages

```bash
apt-cache search ros-indigo
```
