cmake_minimum_required(VERSION 2.8.3)
project(my_controller)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  controller_interface
  pluginlib
  roscpp
)

find_package(Boost REQUIRED COMPONENTS system)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_package( INCLUDE_DIRS src 
	LIBRARIES my_controller  
	CATKIN_DEPENDS pluginlib roscpp
  DEPENDS Boost
)

add_library( my_controller_lib src/my_controller.cpp )
target_link_libraries( my_controller_lib ${catkin_LIBRARIES} )

