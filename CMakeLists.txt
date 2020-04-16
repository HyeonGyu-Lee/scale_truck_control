cmake_minimum_required(VERSION 2.8.3)
project(scale_truck_control)

#add_compile_options(-std=c++11)

#set(OpenCV_INCLUDE_DIRS /usr/share/OpenCV/)
#message("OpenCV_DIR=${OpenCV_INCLUDE_DIRS}")
#message("catkin_DIR=${catkin_INCLUDE_DIRS}")
find_package(catkin REQUIRED COMPONENTS
  cv_bridge roscpp rospy std_msgs message_generation sensor_msgs image_transport geometry_msgs obstacle_detector
  OpenCV REQUIRED
)

add_message_files(
  FILES
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs obstacle_detector
)

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
	

#
# Build libs
#

add_library(lane_detect src/lane_detect.cpp)
target_link_libraries(lane_detect ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(lane_detect ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(LV_node src/nodes/LV_node.cpp)
target_link_libraries(LV_node lane_detect)

add_executable(FV_node src/nodes/FV_node.cpp)
target_link_libraries(FV_node lane_detect)


