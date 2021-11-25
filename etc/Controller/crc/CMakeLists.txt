cmake_minimum_required(VERSION 3.0.2)

project(crc)

set(OpenCV_DIR /usr/share/opencv4/)

find_package(OpenCV REQUIRED)

add_executable(crc main.cpp crc.cpp sock_udp.cpp)

target_include_directories(crc PUBLIC ${CMAKE_SOURCE_DIR}/includes)

target_link_libraries(crc ${OpenCV_LIBS})
