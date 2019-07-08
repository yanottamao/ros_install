# ros_install
ROS Indigo Ubuntu 14.04 Installation Step

## ros base 

### install ros, gazebo

$ sudo apt-get update

$ sudo apt-get upgrade

### add source ros 

$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu trusty main" > /etc/apt/sources.list.d/ros-latest.list'

### add key 

$ wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | sudo apt-key add -
    
## apabila error key not working 

$ sudo apt-key del B01FA116

$ sudo -E apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

### install ros indigo desktop full

$ sudo apt-get update

$ sudo apt-get install ros-indigo-desktop-full

### init rosdep pertama

$ sudo rosdep init

### update rosdep

$ rosdep update

### source bash

$ echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc

$ source ~/.bashrc

### install python-rosinstall

$ sudo apt-get install python-rosinstall

## init catkin workspace

### folder source package

$ mkdir -p ~/catkin_ws/src

### cd folder source

$ cd ~/catkin_ws/src

### init catkin workspace

$ catkin_init_workspace

### build catkin workspace

$ cd ~/catkin_ws

$ catkin_make

### source bash

$ echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc

$ cd

$ source ~/.bashrc

$ cd ~/catkin_ws

## clone library & install library

$ cd ~/catkin_ws/src

### ROS indigo compatible joystick interface

$ git clone https://github.com/occominc/ardrone_joystick.git

### ROS indigo compatible launch scripts

$ git clone https://github.com/occominc/autonavx_ardrone.git

### ROS indigo compatible tum_simulator

$ git clone https://github.com/occominc/tum_simulator.git

### ROS driver for Parrot AR-Drone 1.0 and 2.0 quadrocopters http://wiki.ros.org/ardrone_autonomy

$ git clone https://github.com/AutonomyLab/ardrone_autonomy.git -b indigo-devel

### Wrappers, tools and additional API's for using ROS with Gazebo http://wiki.ros.org/gazebo_ros_pkgs

$ git clone https://github.com/ros-simulation/gazebo_ros_pkgs.git -b indigo-devel

## install dependencies

$ sudo apt-get install ros-indigo-gazebo-ros-pkgs ros-indigo-gazebo-ros-control

## update rosdep 

$ rosdep update

### check dependencies package ros

$ rosdep check --from-paths . --ignore-src --rosdistro indigo

### install dependencies package ros

$ rosdep install --from-paths . --ignore-src --rosdistro indigo -y

## catkin catkin_make build package

$ cd ~/catkin_ws

$ catkin_make
