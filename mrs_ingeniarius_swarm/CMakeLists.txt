cmake_minimum_required(VERSION 3.0.2)
project(mrs_ingeniarius_swarm)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros
  mavros_msgs
  mrs_modules_msgs
  mrs_msgs
  nav_msgs
  quadrotor_msgs
  roscpp
  std_msgs
)

find_package(Boost REQUIRED COMPONENTS system thread)

catkin_package(
  CATKIN_DEPENDS geometry_msgs mavros mavros_msgs mrs_modules_msgs mrs_msgs nav_msgs quadrotor_msgs roscpp std_msgs 
  DEPENDS Boost
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(swarm_cmd_to_mrs_goto src/swarm_cmd_to_mrs_goto.cpp)
target_link_libraries(swarm_cmd_to_mrs_goto ${catkin_LIBRARIES})
add_dependencies(swarm_cmd_to_mrs_goto ${catkin_EXPORTED_TARGETS})