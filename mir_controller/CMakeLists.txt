cmake_minimum_required(VERSION 2.8.3)
project(mir_controller)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  leap_interface
  roscpp
  std_msgs
)

add_compile_options(-std=c++11)

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    leap_interface
    roscpp
    std_msgs
)

###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

add_library(${PROJECT_NAME} src/utilities.cpp src/controlled_robot.cpp)

add_executable(${PROJECT_NAME}_rover_node src/rover.cpp )
target_link_libraries(${PROJECT_NAME}_rover_node ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PROJECT_NAME})

# ----------------- Install -----------------
install(TARGETS ${PROJECT_NAME}_rover_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
