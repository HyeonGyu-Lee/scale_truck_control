cmake_minimum_required(VERSION 2.8.3)
project(scale_truck_control)

#add_compile_options(-std=c++11)

set(ENABLE_DRAFT ON)

add_definitions(-DZMQ_BUILD_DRAFT_API=1)

set(ZeroMQ_DIR /usr/local/lib/)
find_path(ZMQ_INCLUDE_DIR zmq.h)
find_library(ZMQ_LIBRARY NAMES zmq)
set(ZeroMQ_LIBRARIES ${ZMQ_LIBRARY})
set(ZeroMQ_INCLUDE_DIRS ${ZMQ_INCLUDE_DIR})

set(cppzmq_DIR /usr/local/share/cmake/cppzmq)

add_library(cppzmq INTERFACE)

target_link_libraries(cppzmq INTERFACE $<$<CONFIG:Debug>:libzmq>$<$<CONFIG:Release>:libzmq-static>)
target_include_directories(cppzmq INTERFACE ${cppzmq_DIR})
target_compile_definitions(cppzmq INTERFACE ZMQ_BUILD_DRAFT_API=1)

set(OpenCV_DIR /usr/local/lib/cmake/opencv4/)
#set(BOOST_ROOT /usr/include/boost/)
#include_directories(SYSTEM ${Boost_INCLUDE_DIRS})
#message("OpenCV_DIR=${OpenCV_INCLUDE_DIRS}")
#message("catkin_DIR=${catkin_INCLUDE_DIRS}")

find_package(OpenCV 4.4.0 REQUIRED)

find_package(Boost REQUIRED python
  COMPONENTS
    thread
)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp 
    rospy 
    std_msgs
    message_generation
    sensor_msgs
    image_transport
    geometry_msgs
    cv_bridge
    obstacle_detector 
)

add_message_files(
  FILES
  lane_coef.msg
  lane.msg
  xav2lrc.msg
  lrc2xav.msg
  lrc2ocr.msg
  ocr2lrc.msg
)

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
  ${cppzmq_INCLUDE_DIRS}
  ${ZeroMQ_INCLUDE_DIRS}
)

set(PROJECT_LIB_FILES
  src/lane_detect.cpp
  src/lrc.cpp
  src/ScaleTruckController.cpp
  src/sock_udp.cpp
  src/zmq_class.cpp
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

add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}_lib
  pthread
  stdc++
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${Boost_LIBRARIES}
  ${cppzmq_LIBRARIES}
  ${ZeroMQ_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}
  ${PROJECT_NAME}_lib
)

add_library(LRC_lib
  ${PROJECT_LIB_FILES}	
)

add_executable(LRC
  nodes/lrc_node.cpp
)

add_dependencies(LRC
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)

target_link_libraries(LRC_lib
  pthread
  stdc++
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${cppzmq_LIBRARIES}
  ${ZeroMQ_LIBRARIES}
)

target_link_libraries(LRC
  LRC_lib
)

