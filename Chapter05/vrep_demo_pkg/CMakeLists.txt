cmake_minimum_required(VERSION 2.8.3)
project(vrep_demo_pkg)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
  vrep_common
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES vrep_demo_pkg
#  CATKIN_DEPENDS geometry_msgs roscpp std_msgs vrep_common
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(start_stop_scene src/start_stop_scene.cpp)
target_link_libraries(start_stop_scene ${catkin_LIBRARIES})
add_executable(start_stop_scene_with_msg src/start_stop_scene_with_msg.cpp)
target_link_libraries(start_stop_scene_with_msg ${catkin_LIBRARIES})

