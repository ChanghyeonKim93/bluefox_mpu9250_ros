cmake_minimum_required(VERSION 3.5.1)
project(bluefox_mpu9250_ros)
set(CMAKE_BUILD_TYPE Release) # Release, RelWithDebInfo
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -pthread -pg -O2")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}) 
find_package(catkin REQUIRED cmake_modules)
find_package(Eigen REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
)

catkin_package(
  DEPENDS Eigen
  CATKIN_DEPENDS roscpp
  INCLUDE_DIRS serial_comm_ros mpu9250_ros
)

set(INCLUDE_DIRS ${INCLUDE_DIRS} ${CMAKE_CURRENT_BINARY_DIR})
set(INCLUDE_DIRS ${INCLUDE_DIRS} 
  serial_comm_ros
  mpu9250_ros
  common
)

include_directories( ${INCLUDE_DIRS} )
include_directories( ${catkin_INCLUDE_DIRS} )
include_directories( ${Eigen_INCLUDE_DIRS} )

# executables
add_executable(mpu9250_node 
  mpu9250_ros/mpu9250_ros.cpp
  mpu9250_ros/node_mpu9250.cpp
)
target_link_libraries (mpu9250_node ${catkin_LIBRARIES} ${Eigen_LIBRARIES})

add_executable(serial_comm_node 
  serial_comm_ros/crc16.cpp
  serial_comm_ros/serial_comm_ros.cpp
  serial_comm_ros/serial_communicator.cpp
  serial_comm_ros/node_serial_comm.cpp
)
target_link_libraries (serial_comm_node ${catkin_LIBRARIES} ${Eigen_LIBRARIES})