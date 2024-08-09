# ROS Simulation for Pololu Romi Robot

## Overview
This project is a ROS simulation for the Pololu Romi robot. The Romi communicates with a Raspberry Pi running Ubuntu 20.04 and ROS Noetic. It includes models, controllers, and necessary configurations to simulate the Romi robot in a ROS environment. The goal is to provide a platform for testing and developing robotic algorithms.


## Table of Contents
- [ROS Simulation for Pololu Romi Robot](#ros-simulation-for-pololu-romi-robot)
  - [Overview](#overview)
  - [Table of Contents](#table-of-contents)
  - [Parts List](#parts-list)
    - [Robotics Kit](#robotics-kit)
    - [Electronics](#electronics)
    - [Power Supply](#power-supply)
    - [Hardware](#hardware)
    - [3D Printed Parts](#3d-printed-parts)
  - [Images](#images)
  - [Installation](#installation)
  - [Usage](#usage)
  - [System Overview](#system-overview)
- [Additional Documentation](#additional-documentation)


## Parts List

### Robotics Kit
- Pololu Romi Robot Kit

### Electronics
- Raspberry Pi Model 4B
- Raspberry Pi Camera V2.1
- LD19 LiDAR Module
- Compact USB or MicroSD card

### Power Supply
- 6 Rechargable AA Batteries

### Hardware
- M2 Assorted Nylon/Brass Standoffs
- 13 M2x6mm Machine Screws
- 40 Pin Pi Header Extender (Optional)

### 3D Printed Parts
- [3D Printed LiDAR Plate (w/Supports)](docs/files/CAD/lidar_mount.stl)
- [3D Printed Camera Housing](https://www.printables.com/de/model/693396/files)


## Images

<div style="display: flex; justify-content: left; align-items: left;">
   <div style="text-align: center; margin: 10px;">
      <img src="docs/files/romi_robot.jpg" width="200" height="200"/>
      <p>Romi Robot</p>
   </div>

   <div style="text-align: center; margin: 10px;">
      <img src="docs/files/romi_model.gif" width="200" height="200"/>
      <p>Romi Robot Model</p>
   </div>

   <div style="text-align: center; margin: 10px;">
      <img src="docs/files/romi_rviz.png" width="350" height="200"/>
      <p>RViz Display</p>
   </div>

   <div style="text-align: center; margin: 10px;">
      <img src="docs/files/romi_slam.gif" width="350" height="200"/>
      <p>RViz SLAM</p>
   </div>
</div>


## Installation

This workspace should be setup on both a PC and the Romi Raspberry Pi for full usage. The PC will run the ROS master and display the RViz and Gazebo windows. The Raspberry Pi will run the hardware interface with the Romi board, as well as all sensor nodes. 

1. Create or `cd` into a ROS Noetic catkin workspace
   ```console
   mkdir -p ~/catkin_ws/src
   ```

2. Clone this repo in the `src` folder of your catkin workspace:
   ```console
   cd ~/catkin_ws/src
   ```
   ```console
   git clone https://github.com/charlie51135/robot-sim.git
   ```
   
3. Install package dependencies:
   ```
   rosdep install --from-paths src --ignore-src -r -y
   ```

4. Build the catkin workspace:
   ```console
   cd ~/catkin_ws/ && catkin_make
   ```

5. Source packages:
   ```console
   source ~/catkin_ws/devel/setup.bash
   ```

   
## Usage

1. Configure [WiFi settings](docs/ros_ip_setup.md)

2. Launch RViz, the robot desciption and the controllers from a PC:
   ```console
   roslaunch my_robot romi_base.launch
   ```

1. Launch the hardware interface on the Raspberry Pi:
   ```console
   roslaunch romi_base romi_hw.launch
   ```

2. Launch the camera node on the Raspberry Pi:
   ```console
   roslaunch romi_camera romi_camera_node.launch
   ```

1. Launch the LiDAR node on the Raspberry Pi:
   ```console
   roslaunch ldlidar ld19.launch
   ```


## System Overview

<div style="display: flex; justify-content: left; align-items: left;">
   <div style="text-align: center; margin: 10px;">
      <img src="docs/files/system_design.png" width="100%" height="100%"/>
      <p>ROS Nodes Running By Device</p>
   </div>
</div>

</br>

<div style="display: flex; justify-content: left; align-items: left;">
   <div style="text-align: center; margin: 10px;">
      <img src="docs/files/rqt_graph.png" width="100%" height="100%"/>
      <p>ROS RQT Graph</p>
   </div>
</div>

</br>

<div style="display: flex; justify-content: left; align-items: left;">
   <div style="text-align: center; margin: 10px;">
      <img src="docs/files/tf_tree.png" width="100%" height="100%"/>
      <p>ROS TF Tree</p>
   </div>
</div>

</br>
</br>

# Additional Documentation

* [Ros Control Setup](docs/ros_control.md)
* [Raspberry Pi Setup](docs/rpi_setup.md)
* [Camera Setup](docs/rpi_camera.md)
* [Power Consumption](docs/power.md)
* [Accuracy Testing](docs/accuracy_testing.md)

</br>

* [Teleop-Twist-Keyboard Setup](docs/teleop_twist_keyboard.md)
* [Bluetooth Controller Setup](docs/bluetooth_controller.md)
* [Multiple Robots ROS Config](docs/two_robots_rviz.md)
* [Wifi AP](docs/wifi_ap.md)