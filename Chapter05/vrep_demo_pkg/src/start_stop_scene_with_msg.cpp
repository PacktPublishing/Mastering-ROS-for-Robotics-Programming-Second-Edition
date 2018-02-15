/*
 * Copyright (C) 2017, Jonathan Cacace.
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


#include "ros/ros.h"
#include "vrep_common/simRosStartSimulation.h"
#include "vrep_common/simRosStopSimulation.h"
#include "vrep_common/simRosAddStatusbarMessage.h"

int main( int argc, char** argv ) {

	ros::init( argc, argv, "start_stop_vrep_node");
	ros::NodeHandle n;
  ros::ServiceClient start_vrep_client = n.serviceClient<vrep_common::simRosStartSimulation>("/vrep/simRosStartSimulation");
	vrep_common::simRosStartSimulation start_srv;

  ros::ServiceClient stop_vrep_client = n.serviceClient<vrep_common::simRosStopSimulation>("/vrep/simRosStopSimulation");
	vrep_common::simRosStopSimulation stop_srv;

	ros::ServiceClient msg_client = n.serviceClient<vrep_common::simRosAddStatusbarMessage>("/vrep/simRosAddStatusbarMessage");
	vrep_common::simRosAddStatusbarMessage msg_srv;

	ROS_INFO("Starting Vrep simulation...");
	if( start_vrep_client.call( start_srv ) ) {
		if( start_srv.response.result != -1 ) {
				ROS_INFO("Simulation started, wait 5 seconds before stop it!");

				int cnt = 0;
				while( cnt++ < 5 ) {
					//std::cout << "Simulation while stop in " << 6-cnt << " seconds" << std::endl;
					std::stringstream ss;
					ss << "Simulation while stop in " << 6-cnt << " seconds";
					msg_srv.request.message = ss.str();
					if( !msg_client.call( msg_srv ) ) {
						ROS_WARN("Failed to call /vrep/simRosAddStatusbarMessage service"); 					
					}
					
					sleep(1);	
				}
				if( stop_vrep_client.call( stop_srv ) ) {
					if( stop_srv.response.result != -1 ) {
						ROS_INFO("Simulation stopped");	
					}
				}
				else
					ROS_ERROR("Failed to call /vrep/simRosStopSimulation service");
		}

	}
	else
		ROS_ERROR("Failed to call /vrep/simRosStartSimulation service");

	
}
