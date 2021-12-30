# Scale Truck Control

[![Video Label](http://img.youtube.com/vi/wKmWD8BPldw/0.jpg)](https://youtu.be/wKmWD8BPldw?t=0s)

# I. Hardware
>~~~
> High-level Controller - Nvidia Jetson AGX Xavier 8GB
> Low-level Controller  - OpenCR 1.0 (ARM Cortex-M7)
> USB Camera            - ELP-USBFHD04H-BL180
> Lidar                 - RPLidar A3
>~~~

# II. Software
> High-level Controller
>~~~
> Jetpack   : 4.5.1 version - Ubuntu 18.04 LTS
> OpenCV    : 4.4.0 version - include options (GPU, CUDA, CUDNN)
> ZeroMQ    : stable version
> ROS 1     : melodic version
>~~~
>
> Low-level Controller
>~~~
> ros library
>~~~

# III. Demonstration
> Three Scale Truck Platooning
> ~~~
> https://www.youtube.com/watch?v=wKmWD8BPldw
> 
> Intro. 0:00
> Scenario 1. 0:35
> Scenario 2. 1:36
> Case Study: Camera Failure. 2:31
> Emergency Stop. 2:45
> ~~~

# 0. Install Environment
>## 0.1 Jetpack 4.5.1 (ubuntu 18.04 LTS)
>https://developer.nvidia.com/embedded/jetpack

>## 0.2.1 OpenCV 4.4.0
>-Uninstall old version of OpenCV
>~~~
>sudo apt-get purge  libopencv* python-opencv
>sudo apt-get autoremove
>sudo find /usr/local/ -name "*opencv*" -exec rm -i {} \;
>~~~
>
>-Install 4.4.0 version of OpenCV
>~~~
>sudo apt-get update
>sudo apt-get upgrade
>
>sudo apt-get -y install build-essential cmake
>sudo apt-get -y install pkg-config
>sudo apt-get -y install libjpeg-dev libtiff5-dev libpng-dev
>sudo apt-get -y install ffmpeg libavcodec-dev libavformat-dev libswscale-dev libxvidcore-dev libx264-dev libxine2-dev
>sudo apt-get -y install libv4l-dev v4l-utils
>sudo apt-get -y install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev 
>sudo apt-get -y install libgtk-3-dev
>sudo apt-get -y install mesa-utils libgl1-mesa-dri libgtkgl2.0-dev libgtkglext1-dev
>sudo apt-get -y install libatlas-base-dev gfortran libeigen3-dev
>sudo apt-get -y install python3-dev python3-numpy
>~~~
>
>-Download OpenCV 4.4.0 source file
>~~~
>mkdir OpenCV && cd OpenCV
>git clone -b 4.4.0 https://github.com/opencv/opencv
>git clone -b 4.4.0 https://github.com/opencv/opencv_contrib
>cd opencv && mkdir build && cd build
>~~~
>
>-Build OpenCV 4.4.0
>~~~
>cmake -D CMAKE_BUILD_TYPE=RELEASE \
>-D CMAKE_INSTALL_PREFIX=/usr/local \
>-D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules \
>-D WITH_OPENCL=OFF \
>-D WITH_CUDA=ON \
>-D CUDA_ARCH_BIN=7.2 \
>-D CUDA_ARCH_PTX="" \
>-D WITH_CUDNN=ON \
>-D WITH_CUBLAS=ON \
>-D ENABLE_FAST_MATH=ON \
>-D CUDA_FAST_MATH=ON \
>-D OPENCV_DNN_CUDA=ON \
>-D ENABLE_NEON=ON \
>-D WITH_QT=OFF \
>-D WITH_OPENMP=ON \
>-D WITH_OPENGL=ON \
>-D BUILD_TIFF=ON \
>-D WITH_FFMPEG=ON \
>-D WITH_GSTREAMER=ON \
>-D WITH_TBB=ON \
>-D BUILD_TBB=ON \
>-D BUILD_TESTS=OFF \
>-D WITH_V4L=ON \
>-D WITH_LIBV4L=ON \
>-D OPENCV_ENABLE_NONFREE=ON \
>-D INSTALL_C_EXAMPLES=OFF \
>-D INSTALL_PYTHON_EXAMPLES=OFF \
>-D BUILD_NEW_PYTHON_SUPPORT=ON \
>-D BUILD_opencv_python3=TRUE \
>-D OPENCV_GENERATE_PKGCONFIG=ON \
>-D BUILD_EXAMPLES=OFF \
> ..
>~~~
>
>~~~
>sudo make install -j8
>~~~

>## 0.2.2 Environment setup
>~~~
>sudo vim /usr/lib/pkgconfig/opencv.pc
>~~~
>-add the below
>~~~
># Package Information for pkg-config
>prefix=/usr/local
>exec_prefix=${prefix}libdir=${exec_prefix}/lib/aarch64-linux-gnu
>includedir_old=${prefix}/include/opencv4/opencv
>includedir_new=${prefix}/include/opencv4
>
>Name: OpenCV
>Description: Open Source Computer Vision Library
>Version: 4.4.0
>Libs: -L${exec_prefix}/lib/aarch64-linux-gnu -lopencv_dnn -lopencv_gapi -lopencv_highgui -lopencv_ml -lopencv_objdetect -lopencv_photo -lopencv_stitching -lopencv_video -lopencv_calib3d -lopencv_features2d -lopencv_flann -lopencv_videoio -lopencv_imgcodecs -lopencv_imgproc -lopencv_core
>Libs.private: -ldl -lm -lpthread -lrt
>Cflags: -I${includedir_old} -I${includedir_new}
>~~~

>## 0.2.3 Jetson Stats
>~~~
>sudo -H pip3 install jetson-stats
>jeson_release
>~~~

>## 0.3.1 ZeroMQ (cppzmq)
>-Install and Build of ZeroMQ for cpp
>~~~
>sudo apt update
>sudo apt upgrade
>sudo apt remove libzmq3-dev
>sudo apt remove cmake
>~~~
>http://github.com/zeromq/cppzmq
>~~~
># Upgrade cmake (CMake 3.10 or higher is required)
>wget https://github.com/Kitware/CMake/archive/master.zip
>unzip master.zip
>rm master.zip
>cd CMake-master
>./bootstrap
>sudo make -j8
>sudo make install
>sudo ldconfig
>cmake --version
>cd ../
>
>wget https://download.libsodium.org/libsodium/releases/libsodium-1.0.18-stable.tar.gz
>tar -xvf libsodium-*
>rm *libsodium-1.0.18-stable.tar.gz
>cd libsodium-stable
>./configure
>sudo make clean
>sudo make -j8
>sudo make install 
>sudo ldconfig
>cd ../
>
># Build, check, and install the latest version of ZeroMQ
>wget https://github.com/zeromq/libzmq/archive/master.zip
>unzip master.zip
>rm master.zip
>cd libzmq-master
>./autogen.sh 
>./configure --with-libsodium
>mkdir build
>cd build && cmake .. -DENABLE_DRAFTS=ON
>sudo make -j8 install
>sudo ldconfig
>cd ../../
>
># Now install CPPZMQ
>wget https://github.com/zeromq/cppzmq/archive/master.zip
>unzip master.zip
>rm master.zip
>cd cppzmq-master
>mkdir build
>cd build && cmake .. -DENABLE_DRAFTS=ON
>sudo make -j8 install
>sudo ldconfig
>cd ../../
>~~~

>## 0.3.2 Environment setup
>-Setup the path
>~~~
>sudo cp -R /usr/local/lib/* /usr/lib
>~~~

# 1. Install ROS (melodic)
http://wiki.ros.org/melodic/Installation/Ubuntu  
cd ~/catkin_ws/src
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
# 2. Install ROS Packages
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
>git clone -b kinetic-devel https://github.com/ros-perception/laser_filters.git 
>```
>## 2.7 vison_opencv (vision_opencv, image_geometry, cv_bridge)
>```
>git clone https://github.com/ros-perception/vision_opencv.git
>```

# 3. Install module & Environment setup
- melodic is ros-version (18.04 LTS)
>## 3.1 cv_bridge Setup
>- 3.1.1
>```
>vim ~/catkin_ws/src/vision_opencv/cv_bridge/CMakelist.txt
>```
>-fix the below
>```
>--find_package(Boost REQUIRED python37)
>++find_package(Boost REQUIRED python)
>```
>- 3.1.2
>```
>vim ~/catkin_ws/src/vision_opencv/cv_bridge/src/module.hpp
>```
>-fix the below
>```
>--static void * do_numpy_import( )
>++static void do_numpy_import( )
>--return nullptr;
>```

>## 3.2 rosserial_Arduino Module
>- communicate with OpenCR 1.0
>```
>sudo apt-get install ros-melodic-rosserial-arduino   
>sudo apt-get install ros-melodic-rosserial   
>```
>## 3.3 armadillo , qtbase5 Module
>- Python UI Module -y
>```
>sudo apt-get install libarmadillo-dev
>sudo apt-get qtbase5-dev 
> ```
>## 3.4 alias command Setup
>```
>sudo vim ~/.bashrc
>```
>-add the below
>```
>++source ~/catkin_ws/devel/setup.bash
>++alias cw='cd ~/catkin_ws'
>++alias cs='cd ~/catkin_ws/src'
>++alias cm='cd ~/catkin_ws && catkin_make'
>++alias cb='source ~/catkin_ws/devel/setup.bash'
>++alias sb='source ~/.bashrc'
>```
>```
>source ~/.bashrc
>```
>
>## 3.5 ros package build
>```
>cm
>```

# 4. Run
>## 4.1 rosbag test
>- LV(Leading Vehicle) rosbag file download (3.57G)
>```
>curl -c ./cookie -s -L "https://drive.google.com/uc?export=download&id=1ATriInXrn-BYf4-K1rT65GYfE_tnerWV" > /dev/null
>curl -Lb ./cookie "https://drive.google.com/uc?export=download&confirm=`awk '/download/ {print $NF}' ./cookie`&id=1ATriInXrn-BYf4-K1rT65GYfE_tnerWV" -o "LV-08-13.bag"
>```
>
>- FV(Following Vehicle) rosbag file download (3.66G)
>```
>curl -c ./cookie -s -L "https://drive.google.com/uc?export=download&id=1Uo-cWdeLFKnIperpsSOJ3AoVdohLcVgp" > /dev/null
>curl -Lb ./cookie "https://drive.google.com/uc?export=download&confirm=`awk '/download/ {print $NF}' ./cookie`&id=1Uo-cWdeLFKnIperpsSOJ3AoVdohLcVgp" -o "FV-08-13.bag"
>```
>
>## 4.2 Rosbag run
>```
>rosbag play [rosbag file name].bag
>```
>## 4.3 Ros Launch
>```
>roslaunch scale_truck_control control_test.launch
>```
