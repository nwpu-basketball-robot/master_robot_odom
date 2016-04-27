cmake_minimum_required(VERSION 2.8.3)
project(basketball_odom)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  basketball_msgs
  geometry_msgs
  nav_msgs
  roscpp
  std_msgs
  tf
)


catkin_package(
  INCLUDE_DIRS include
 LIBRARIES basketball_odom
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp std_msgs basketball_msgs
  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_executable(robot_odom src/basketball_odom.cpp)
target_link_libraries(robot_odom ${catkin_LIBRARIES})
add_dependencies(robot_odom basketball_msgs_gencpp)
