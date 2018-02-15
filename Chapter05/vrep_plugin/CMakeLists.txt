# This file is part of the ROS PLUGIN for V-REP
# 
# Copyright 2006-2016 Coppelia Robotics GmbH. All rights reserved. 
# marc@coppeliarobotics.com
# www.coppeliarobotics.com
# 
# Copyright 2017 Jonathan Cacace. All rights reserved. 
# jonathan.cacace@gmail.com
# 
# 
# The ROS PLUGIN is licensed under the terms of GNU GPL:
# 
# -------------------------------------------------------------------
# The ROS PLUGIN is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
# 
# THE ROS PLUGIN IS DISTRIBUTED "AS IS", WITHOUT ANY EXPRESS OR IMPLIED
# WARRANTY. THE USER WILL USE IT AT HIS/HER OWN RISK. THE ORIGINAL
# AUTHORS AND COPPELIA ROBOTICS GMBH WILL NOT BE LIABLE FOR DATA LOSS,
# DAMAGES, LOSS OF PROFITS OR ANY OTHER KIND OF LOSS WHILE USING OR
# MISUSING THIS SOFTWARE.
# 
# See the GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with the ROS PLUGIN.  If not, see <http:#www.gnu.org/licenses/>.
# -------------------------------------------------------------------
#
# This file was automatically created for V-REP release V3.3.0 on February 19th 2016

cmake_minimum_required(VERSION 2.8.3)
project(vrep_plugin)

find_package(catkin REQUIRED COMPONENTS
  roscpp image_transport tf vrep_common
)

find_package( OpenCV REQUIRED )
catkin_package(
               CATKIN_DEPENDS roscpp rosconsole std_msgs sensor_msgs cv_bridge image_transport tf brics_actuator roslib
               DEPENDS roscpp rosconsole std_msgs sensor_msgs cv_bridge image_transport tf brics_actuator roslib)

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

link_directories("/opt/ros/groovy/lib")
link_directories("/opt/ros/indigo/lib")
link_directories("/opt/ros/hydro/lib")
link_directories("/opt/ros/kinetic/lib")

#find_package( OpenCV )
#include_directories(include ${catkin_INCLUDE_DIRS} /opt/ros/groovy/share/OpenCV)

add_library(v_repExtRos src/vrep_plugin.cpp src/luaFunctionData.cpp src/luaFunctionDataItem.cpp src/v_repLib.cpp src/ROS_server.cpp src/vrepSubscriber.cpp)

add_dependencies(v_repExtRos ${catkin_EXPORTED_TARGETS})
target_link_libraries(v_repExtRos
    pthread
    dl
    roslib
    rosconsole
    rostime
    rospack
    roscpp_serialization
    roscpp
    tf
    image_transport
    boost_system
   ${catkin_LIBRARIES}
${OpenCV_LIBS}
)
