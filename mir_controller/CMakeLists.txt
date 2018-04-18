cmake_minimum_required(VERSION 2.8.3)
project(mir_controllers)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  geometry_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  pluginlib
  geometric_shapes
  leap_interface
)


add_compile_options(-std=c++11)

find_package(Eigen3 REQUIRED)

# Eigen 3.2 (Wily) only provides EIGEN3_INCLUDE_DIR, not EIGEN3_INCLUDE_DIRS
if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()



find_package(Boost REQUIRED system filesystem date_time thread)

catkin_package(
  CATKIN_DEPENDS
    moveit_core
    moveit_ros_planning_interface
    interactive_markers
  DEPENDS
    EIGEN3
)



###########
## Build ##
###########

include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

include_directories(include ${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})

add_library(utilities src/utilities.cpp)
add_library(controlled_robot src/controlled_robot.cpp)

add_executable(robotic_arm src/robotic_arm.cpp include/controlled_robot.h include/robotic_arm.h include/utilities.h)
target_link_libraries(robotic_arm ${catkin_LIBRARIES} ${Boost_LIBRARIES} utilities controlled_robot)
install(TARGETS robotic_arm DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(rover src/rover.cpp include/controlled_robot.h include/rover.h include/utilities.h)
target_link_libraries(rover ${catkin_LIBRARIES} ${Boost_LIBRARIES} utilities controlled_robot)
install(TARGETS rover DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
