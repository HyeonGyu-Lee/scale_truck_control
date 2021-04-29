cmake_minimum_required(VERSION 2.8.3)
project(scale_truck_control)

#add_compile_options(-std=c++11)

set(OpenCV_DIR /usr/share/opencv4/)
#set(BOOST_ROOT /usr/include/boost/)
#include_directories(SYSTEM ${Boost_INCLUDE_DIRS})
#message("OpenCV_DIR=${OpenCV_INCLUDE_DIRS}")
#message("catkin_DIR=${catkin_INCLUDE_DIRS}")

find_package(Boost REQUIRED
  COMPONENTS
    thread
)

find_package(catkin REQUIRED
  COMPONENTS
    cv_bridge
    roscpp 
    rospy 
    std_msgs
    message_generation
    sensor_msgs
    image_transport
    geometry_msgs
    obstacle_detector
    
)

find_package(OpenCV REQUIRED)

#add_message_files(
#  FILES
#)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    rospy
    std_msgs
    geometry_msgs
    obstacle_detector
  DEPENDS
    Boost
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

set(PROJECT_LIB_FILES
  src/lane_detect.cpp					src/ScaleTruckController.cpp
)

#
# Build libs
#

add_library(${PROJECT_NAME}_lib
  ${PROJECT_LIB_FILES}	
)

add_executable(${PROJECT_NAME}
  nodes/control_node.cpp
)

target_link_libraries(${PROJECT_NAME}_lib
  pthread
  stdc++
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${Boost_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}
  ${PROJECT_NAME}_lib
)

add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)
