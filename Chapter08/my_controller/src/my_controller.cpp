/*
 * Copyright (C) 2017, Jonathan Cacace

 * Email id : jonathan.cacace@gmail.com

 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.

*/


#include "my_controller.h"


namespace my_controller_ns {

//Controller initialization
  bool MyControllerClass::init(hardware_interface::PositionJointInterface* hw, ros::NodeHandle &nh)
  {
		//Retrieve the joint object to control
		std::string joint_name;
		if( !nh.getParam( "joint_name", joint_name ) ) {
			ROS_ERROR("No joint_name specified");
			return false;
		}
    joint_ = hw->getHandle(joint_name); 
    return true;
  }

//Controller startup
  void MyControllerClass::starting(const ros::Time& time) {

		//Get initial position to use in the control procedure
		init_pos_ = joint_.getPosition();
	}

//Controller running
  void MyControllerClass::update(const ros::Time& time, const ros::Duration& period)
  {
		//---Perform a sinusoidal motion for joint shoulder_pan_joint
		double dpos = init_pos_ + 10 * sin(ros::Time::now().toSec());
		double cpos = joint_.getPosition();
		joint_.setCommand( -10*(cpos-dpos)); //Apply command to the selected joint
		//---
  }

//Controller exiting
  void MyControllerClass::stopping(const ros::Time& time) { }

}

//Register the plugin: PLUGINLIB_EXPORT_CLASS(my_namespace::MyPlugin, base_class_namespace::PluginBaseClass)
PLUGINLIB_EXPORT_CLASS(my_controller_ns::MyControllerClass, controller_interface::ControllerBase);
