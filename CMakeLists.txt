cmake_minimum_required(VERSION 2.8.3)
project(ft_scservo_driver)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  diagnostic_updater
  hardware_interface
  roscpp
  std_srvs
)

find_package(Boost REQUIRED COMPONENTS thread)

################################################
## Declare ROS messages, services and actions ##
################################################

generate_dynamic_reconfigure_options(
  cfg/Servo.py
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES scservo_ros
  CATKIN_DEPENDS diagnostic_updater dynamic_reconfigure hardware_interface roscpp std_srvs
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(scservo src/scservo.c)
add_library(scservo_ros src/scservo_driver.cpp src/scservo_handle.cpp)

add_executable(scservo_node src/scservo_node.cpp)
add_executable(scservo_scan src/scservo_scan.c)

add_dependencies(scservo_ros ${PROJECT_NAME}_gencfg)

target_link_libraries(scservo_node scservo_ros ${catkin_LIBRARIES})
target_link_libraries(scservo_ros scservo ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(scservo_scan scservo)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

#############
## Install ##
#############

install(TARGETS scservo scservo_node scservo_ros scservo_scan
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME} include/scservo
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
)
