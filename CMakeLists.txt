cmake_minimum_required(VERSION 2.8.7)

project(ros_inemo_m1_imu)

#########################################################
## Options
option( DEBUG_ACTIVE "Enable Debug build" OFF )

if(DEBUG_ACTIVE)
    MESSAGE( "Debug compilation active" )
    set(ROS_BUILD_TYPE Debug)
    set(CMAKE_BUILD_TYPE Debug)
else()
    MESSAGE( "Release compilation active" )
    set(ROS_BUILD_TYPE Release)
    set(CMAKE_BUILD_TYPE Release)
endif()
#########################################################

#########################################################
# INCLUDES
set(INEMO_INC_PATH
    ${ros_inemo_m1_imu_SOURCE_DIR}/src/driver/include)

include_directories(${INEMO_INC_PATH})
#########################################################

#########################################################
# SOURCES
file(GLOB_RECURSE inemo_m1_SRC *.cpp)
# HEADERS
file(GLOB_RECURSE inemo_m1_HDR *.h)
#########################################################

#########################################################
#CATKIN
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

include_directories(${catkin_INCLUDE_DIRS})
#########################################################


#########################################################
# EXECUTABLE
add_executable(
    inemo_m1_node
    ${inemo_m1_HDR}
    ${inemo_m1_SRC}
)

target_link_libraries(
    inemo_m1_node
    ${catkin_LIBRARIES} pthread
)
#########################################################
