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
 *
*/

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

using namespace std;

int main(int argc, char **argv) {

	ros::init(argc, argv, "seven_dof_arm_planner");
	ros::NodeHandle node_handle;
	ros::AsyncSpinner spinner(1);

	spinner.start();

	moveit::planning_interface::MoveGroupInterface group("arm");
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;


	string line;

	//Waiting for scene initialization
	sleep(2);


	//--- objects into the scene
	moveit::planning_interface::PlanningSceneInterface current_scene;
	geometry_msgs::Pose pose;

	//---Add grasping object to the scene
	shape_msgs::SolidPrimitive primitive;
	primitive.type = primitive.BOX;
	primitive.dimensions.resize(3);
	primitive.dimensions[0] = 0.03;
	primitive.dimensions[1] = 0.03;
	primitive.dimensions[2] = 0.08;

	moveit_msgs::CollisionObject grasping_object;
	grasping_object.id = "grasping_object";
	pose.orientation.w = 1.0;
	pose.position.y =  0.0;
	pose.position.x =  0.33;
	pose.position.z =  0.35;

	grasping_object.primitives.push_back(primitive);
	grasping_object.primitive_poses.push_back(pose);
	grasping_object.operation = grasping_object.ADD;
	grasping_object.header.frame_id = "base_link";

	primitive.dimensions.resize(3);
	primitive.dimensions[0] = 0.3;
	primitive.dimensions[1] = 0.5;
	primitive.dimensions[2] = 0.32;
	moveit_msgs::CollisionObject grasping_table;
	grasping_table.id = "grasping_table";
	pose.position.y =  0.0;
	pose.position.x =  0.46;
	pose.position.z =  0.15;
	grasping_table.primitives.push_back(primitive);
	grasping_table.primitive_poses.push_back(pose);
	grasping_table.operation = grasping_object.ADD;
	grasping_table.header.frame_id = "base_link";
	std::vector<moveit_msgs::CollisionObject> collision_objects;
	collision_objects.push_back(grasping_object);
	collision_objects.push_back(grasping_table);
	//---


	// Once all of the objects (in this case just one) have been added to the
	// vector, we tell the planning scene to add our new box
	current_scene.addCollisionObjects(collision_objects);

	moveit::planning_interface::MoveGroupInterface::Plan my_plan;
	const robot_state::JointModelGroup *joint_model_group =
  group.getCurrentState()->getJointModelGroup("arm");




	//---approaching
	geometry_msgs::Pose target_pose;
	target_pose.orientation.x = 0;
	target_pose.orientation.y = 0;
	target_pose.orientation.z = 0;
	target_pose.orientation.w = 1;
	target_pose.position.y = 0.0;
	target_pose.position.x = 0.32;
	target_pose.position.z = 0.35;
	group.setPoseTarget(target_pose);
	group.move();
	sleep(2);

	//---grasping
	target_pose.position.y = 0.0;
	target_pose.position.x = 0.34;
	target_pose.position.z = 0.35;
	group.setPoseTarget(target_pose);
	group.move();

	//---attach object to the robot
	moveit_msgs::AttachedCollisionObject attacched_object;
	attacched_object.link_name = "grasping_frame";
	attacched_object.object = grasping_object;
	current_scene.applyAttachedCollisionObject( attacched_object );
	sleep(2);

	//---move far away from the grasping position
	target_pose.position.y = 0.0;
	target_pose.position.x = 0.34;
	target_pose.position.z = 0.4;
	group.setPoseTarget(target_pose);
	group.move();
	sleep(2);

	//---picking
	target_pose.orientation.x = -1;
	target_pose.orientation.y = 0;
	target_pose.orientation.z = 0;
	target_pose.orientation.w = 0;
	target_pose.position.y = -0.1;
	target_pose.position.x = 0.34;
	target_pose.position.z = 0.4;
	group.setPoseTarget(target_pose);
	group.move();
	//---
	
	target_pose.position.y = -0.1;
	target_pose.position.x = 0.34;
	target_pose.position.z = 0.35;
	group.setPoseTarget(target_pose);
	group.move();

	//---remove object from robot's body
	grasping_object.operation = grasping_object.REMOVE;
	attacched_object.link_name = "grasping_frame";
	attacched_object.object = grasping_object;
	current_scene.applyAttachedCollisionObject( attacched_object );
	target_pose.position.y = -0.1;
	target_pose.position.x = 0.32;
	target_pose.position.z = 0.35;
	group.setPoseTarget(target_pose);
	group.move();

	ros::shutdown();

}
