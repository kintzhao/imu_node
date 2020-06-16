cmake_minimum_required(VERSION 2.8.3)
project(imu_node)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

SET(CMAKE_CXX_FLAGS "$ENV{CXXFLAGS} -O3 -Wall -std=c++11")
SET(CMAKE_C_FLAGS "$ENV{CFLAGS} -O3 -Wall")

find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf
  roslib
)
 
 
catkin_package(
   INCLUDE_DIRS include
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)
 
 message(${EIGEN3_INCLUDE_DIR})
include_directories(
  include 
  include/imu
  include/libSerial
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

 
add_library(serial  src/libserial/SerialPort.cpp
                    src/libserial/SerialStream.cpp 
                    src/libserial/SerialStreamBuf.cpp 
                    
                    )

## Declare a C++ library
add_library(imu_packet
  src/imu/packet.c
  src/imu/imu_data_decode.c
)

add_executable(imu_node src/imu_node.cpp)
# 
target_link_libraries(imu_node
  serial imu_packet
  ${catkin_LIBRARIES}
)
 
