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
# INCLUDES

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
    src/driver/src/inemo_m1_driver.cpp
)
#########################################################

#########################################################
# EXECUTABLE
add_executable(
	inemo_m1_node 
        ${inemo_m1_SRC}
)

target_link_libraries(
	inemo_m1_node
  	${catkin_LIBRARIES} pthread
)
#########################################################
