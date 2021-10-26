cmake_minimum_required(VERSION 3.0.2)
project(wloam)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS "-Wall -Wextra -DPCL_NO_PRECOMPILE")
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_DEBUG_POSTFIX "_d")

find_package(catkin REQUIRED COMPONENTS
  message_generation
  eigen_conversions
  gps_common
  roscpp
  rospy
  nav_msgs
  sensor_msgs
  apa_msgs
  tf
  tf_conversions
  visualization_msgs
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED QUIET COMPONENTS common io)
find_package(GTSAM REQUIRED)

add_message_files(
  FILES
  CloudInfo.msg
  CustomPoint.msg
  CustomMsg.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
)

catkin_package(
  # INCLUDE_DIRS
  # LIBRARIES wloam
  CATKIN_DEPENDS 
    message_runtime
    eigen_conversions
    gps_common
    roscpp
    rospy
    nav_msgs
    sensor_msgs
    apa_msgs
    tf
    tf_conversions
    visualization_msgs
  DEPENDS Boost EIGEN3 OpenCV PCL GTSAM
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
)

add_subdirectory(src)