# Scale Truck Control

# Hardware
>Nvidia Jetson Xavier (18.04 LTS - JetPack 4.5.1)   
>USB Camera (Camera)   
>RPLidar A3 (Lidar)   
>OpenCR 1.0 (ARM)

# 1. Install ROS (melodic)
http://wiki.ros.org/melodic/Installation/Ubuntu

>## 1.1 Setup your sources.list
>```
>sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
>```
>## 1.2 Set up your keys
>```
>sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
>```
>## 1.3 Installation
>```
>sudo apt update   
>sudo apt install ros-melodic-desktop-full
>```
>## 1.4 Environment setup
>```
>echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
>source ~/.bashrc
>source /opt/ros/melodic/setup.bash
>```
>## 1.5 Dependencies for building packages
>### python 2.7
>```
>sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
>sudo apt install python-rosdep
>sudo rosdep init
>rosdep update
>```
>### python 3.6
>```
>sudo apt-get install python3-pip python3-yaml
>sudo pip3 install rospkg catkin_pkg
>```
>## 1.6 Create a ROS Workspace
>```
>mkdir -p ~/catkin_ws/src
>cd ~/catkin_ws/
>catkin_make
>```
>### python3 option
>```
>catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3
>```
# 2. Setup ROS Packages
>## 2.1 scale_truck_control
>```
>git clone https://github.com/HyeonGyu-Lee/scale_truck_control.git 
>```
>## 2.2 geometry_msgs
>```
>git clone https://github.com/ros/common_msgs.git
>```
>## 2.3 usb_cam
>```
>git clone https://github.com/ros-drivers/usb_cam.git
>```
>## 2.4 ros_rplidar
>```
>git clone https://github.com/robopeak/rplidar_ros.git
>```
>## 2.5 obstacle_detector
>```
>git clone https://github.com/tysik/obstacle_detector.git
>```
>## 2.6 laser_filters
>```
>git clone https://github.com/ros-perception/laser_filters.git 
>```

# 3. Install module
>## 3.1 cv_bridge
>sudo apt-get install ros-melodic-cv-bridge   
>sudo apt-get install ros-melodic-vision-opencv   
> - melodic is ros-version-name
>## 3.2 OpenCV Version
> ```
> cd /opt/ros/melodic/share/cv_bridge/cmake
> sudo vim cv_bridgeConfig.cmake
> --set(_include_dirs "include;/usr/include;/usr/include/opencv
> ++set(_include_dirs "include;/usr/include;/usr/include/opencv4
> ```
> ```
> cd ~/catkin_ws/src/scale_truck_control
> vim CMakeLists.txt
> set OpenCV_DIR /usr/share/opencv4
> ```
>## 3.3 rosserial_Arduino
>sudo apt-get install ros-melodic-rosserial-arduino   
>sudo apt-get install ros-melodic-rosserial   
> - melodic is ros-version-name
> ## 3.4 armadillo , qtbase5 
> sudo apt-get install libarmadillo-dev
> sudo apt-get qtbase5-dev 



# 4. Run

