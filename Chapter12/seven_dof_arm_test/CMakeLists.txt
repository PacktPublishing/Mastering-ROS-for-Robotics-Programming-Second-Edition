cmake_minimum_required(VERSION 2.8.3)
project(seven_dof_arm_test)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  interactive_markers
  moveit_core
  moveit_ros_perception
  moveit_ros_planning_interface
  pluginlib
  roscpp
  std_msgs
)

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

###############################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES seven_dof_arm_test
#  CATKIN_DEPENDS cmake_modules interactive_markers moveit_core moveit_ros_perception moveit_ros_planning_interface pluginlib roscpp std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)


## Declare a cpp executable
add_executable(test_random_node src/test_random.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(test_random_node seven_dof_arm_test_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(test_random_node
   ${catkin_LIBRARIES}
 )


include_directories(
  ${catkin_INCLUDE_DIRS}
)


## Declare a cpp executable
add_executable(test_custom_node src/test_custom.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(test_custom_node seven_dof_arm_test_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(test_custom_node
   ${catkin_LIBRARIES}
 )

## Declare a cpp executable
add_executable(add_collision_object src/add_collision_object.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(add_collision_object seven_dof_arm_test_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(add_collision_object
   ${catkin_LIBRARIES}
 )



## Declare a cpp executable
add_executable(remove_collision_objct src/remove_collision_objct.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(remove_collision_objct seven_dof_arm_test_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(remove_collision_objct
   ${catkin_LIBRARIES}
)

## Declare a cpp executable
add_executable(pick_place src/pick_place.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(pick_place seven_dof_arm_test_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(pick_place
   ${catkin_LIBRARIES}
)

## Declare a cpp executable
add_executable(check_collision src/check_collision.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(check_collision seven_dof_arm_test_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(check_collision
   ${catkin_LIBRARIES}
)

## Declare a cpp executable
add_executable(attach_detach_object src/attach_detach_object.cpp)
target_link_libraries(attach_detach_object ${catkin_LIBRARIES} )


## Declare a cpp executable
add_executable(pick_place_action src/pick_place_action.cpp)
target_link_libraries(pick_place_action ${catkin_LIBRARIES} )
