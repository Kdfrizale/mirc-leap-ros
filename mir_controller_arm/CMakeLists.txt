cmake_minimum_required(VERSION 2.8.3)
project(mir_controller_arm)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  leap_interface
  mir_controller
  moveit_core
  moveit_ros_planning_interface
  roscpp
)

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    leap_interface
    moveit_core
    moveit_ros_planning_interface
)

###########
## Build ##
###########
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_node src/robotic_arm.cpp )
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})
install(TARGETS ${PROJECT_NAME}_node DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
