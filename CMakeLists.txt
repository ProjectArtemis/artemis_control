cmake_minimum_required(VERSION 2.8.3)
project(artemis_control)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  sensor_msgs
  cmake_modules
)

find_package(Eigen REQUIRED)

add_message_files(
  DIRECTORY msg
  FILES
  Actuators.msg
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  INCLUDE_DIRS include ${Eigen_INCLUDE_DIRS}
  LIBRARIES trajectory_controller
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp sensor_msgs
  DEPENDS Eigen
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_library(trajectory_controller
  src/trajectory_controller.cpp
)

target_link_libraries(trajectory_controller ${catkin_LIBRARIES})
add_dependencies(trajectory_controller ${catkin_EXPORTED_TARGETS})

add_executable(trajectory_controller_node src/trajectory_controller_node.cpp)
add_dependencies(trajectory_controller_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(trajectory_controller_node
  trajectory_controller ${catkin_LIBRARIES})


install(TARGETS trajectory_controller
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS trajectory_controller_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
