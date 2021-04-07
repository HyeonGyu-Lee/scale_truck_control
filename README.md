# install ROS
http://wiki.ros.org/melodic/Installation/Ubuntu

# Setup Env.
http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment
build option : catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3

# install python3 Env.
sudo apt-get install python3-pip python3-yaml
sudo pip3 install rospkg catkin_pkg

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

## rplidar_plat
git clone https://github.com/hanadul/rplidar_plat.git

# Install module
## cv_bridge
sudo apt-get install ros-melodic-cv-bridge   
sudo apt-get install ros-melodic-vision-opencv   
 - melodic is ros-version-name

## rosserial_Arduino
sudo apt-get install ros-melodic-rosserial-arduino   
sudo apt-get install ros-melodic-rosserial   
 - melodic is ros-version-name
