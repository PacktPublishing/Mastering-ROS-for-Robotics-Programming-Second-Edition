cmake_minimum_required(VERSION 2.8.3)
project(rviz_teleop_commander)

find_package(catkin REQUIRED COMPONENTS rviz)
find_package(Qt5 COMPONENTS Core Widgets REQUIRED)
set(QT_LIBRARIES Qt5::Widgets)




add_definitions(-DQT_NO_KEYWORDS)

catkin_package(
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS roscpp
                   rviz
)

include_directories(include 
	${catkin_INCLUDE_DIRS} 
	${Boost_INCLUDE_DIRS}
)

link_directories(
	${catkin_LIBRARY_DIRS}
  ${Boost_LIBRARY_DIRS}
)

add_definitions(-DQT_NO_KEYWORDS)

QT5_WRAP_CPP(MOC_FILES
  src/teleop_pad.h 
  OPTIONS -DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED -DBOOST_LEXICAL_CAST_INCLUDED 
)

set(SOURCE_FILES
  src/teleop_pad.cpp 
  ${MOC_FILES}
)

## An rviz plugin is just a shared library, so here we declare the
## library to be called ``${PROJECT_NAME}`` (which is
## "rviz_plugin_tutorials", or whatever your version of this project
## is called) and specify the list of source files we collected above
## in ``${SOURCE_FILES}``.
add_library(${PROJECT_NAME} ${SOURCE_FILES})

## Link the library with whatever Qt libraries have been defined by
## the ``find_package(Qt4 ...)`` line above, and with whatever libraries
## catkin has included.
##
## Although this puts "rviz_plugin_tutorials" (or whatever you have
## called the project) as the name of the library, cmake knows it is a
## library and names the actual file something like
## "librviz_plugin_tutorials.so", or whatever is appropriate for your
## particular OS.
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${catkin_LIBRARIES})

## Install rules
install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES 
  plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

