# Hardware
Nvidia Jetson Xavier (18.04 LTS - JetPack 4.5.1)   
USB Camera (Camera)   
RPLidar A3 (Lidar)   
OpenCR 1.0 (ARM)

# install ROS (melodic)
http://wiki.ros.org/melodic/Installation/Ubuntu

# Setup Env.
http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment   

# ROS_ws/src
## scale_truck_control
git clone https://github.com/HyeonGyu-Lee/scale_truck_control.git 

## geometry_msgs
git clone https://github.com/ros/common_msgs.git

## usb_cam
git clone https://github.com/ros-drivers/usb_cam.git

## ros_rplidar
git clone https://github.com/robopeak/rplidar_ros.git

## obstacle_detector
git clone https://github.com/tysik/obstacle_detector.git

## laser_filters
git clone https://github.com/ros-perception/laser_filters.git 

# Install module
## cv_bridge
sudo apt-get install ros-melodic-cv-bridge   
sudo apt-get install ros-melodic-vision-opencv   
 - melodic is ros-version-name

## rosserial_Arduino
sudo apt-get install ros-melodic-rosserial-arduino   
sudo apt-get install ros-melodic-rosserial   
 - melodic is ros-version-name
