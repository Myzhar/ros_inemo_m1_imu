cmake_minimum_required(VERSION 2.8.8)

project(st_inemo_m1_imu)

set(ROS_BUILD_TYPE Debug)
set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  serial
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    serial
    std_msgs
    sensor_msgs
    geometry_msgs
)

#########################################################
# Qt5
set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(Qt5Core REQUIRED)
#########################################################

#########################################################
# INCLUDES
set(qt_header_path
     src/driver/include
)

set(inemo_m1_INC_DIRS
    src/
    src/driver/include/
)

# Specify locations of header files.
include_directories(
	${catkin_INCLUDE_DIRS} 
	${inemo_m1_INC_DIRS}
)
#########################################################

#########################################################
# SOURCES
set(inemo_m1_SRC
    src/inemo_m1_node.cpp
    src/driver/include/inemo_m1_driver.h # needed to make AUTOMOC work
    src/driver/src/inemo_m1_driver.cpp
)
#########################################################

#########################################################
# EXECUTABLE
add_executable(
	inemo_m1_node 
        ${inemo_m1_SRC}
)

qt5_use_modules(inemo_m1_node Core)

target_link_libraries(
	inemo_m1_node
  	${catkin_LIBRARIES}
)
#########################################################
