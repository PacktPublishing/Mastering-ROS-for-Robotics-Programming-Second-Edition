#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <moveit_msgs/PickupAction.h>

//#include <moveit/move_group_interface/move_group.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>


int main (int argc, char **argv)
{
	ros::init(argc, argv, "seven_dof_arm_planner");
	ros::NodeHandle node_handle;
	ros::AsyncSpinner spinner(1);

	spinner.start();

	moveit::planning_interface::MoveGroupInterface group("arm");
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

	//Waiting for scene initialization
	sleep(2);


	//--- objects into the scene
	moveit::planning_interface::PlanningSceneInterface current_scene;
	moveit_msgs::CollisionObject grasping_object;
	grasping_object.id = "grasping_object";

	//---Add grasping object to the scene
	shape_msgs::SolidPrimitive primitive;
	primitive.type = primitive.BOX;
	primitive.dimensions.resize(3);
	primitive.dimensions[0] = 0.03;
	primitive.dimensions[1] = 0.03;
	primitive.dimensions[2] = 0.08;

	geometry_msgs::Pose pose;
	pose.orientation.w = 1.0;
	pose.position.y =  0.0;
	pose.position.x =  0.33;
	pose.position.z =  0.35;

	grasping_object.primitives.push_back(primitive);
	grasping_object.primitive_poses.push_back(pose);
	grasping_object.operation = grasping_object.ADD;
	grasping_object.header.frame_id = "base_link";
	std::vector<moveit_msgs::CollisionObject> collision_objects;
	collision_objects.push_back(grasping_object);
	//---


	// Once all of the objects (in this case just one) have been added to the
	// vector, we tell the planning scene to add our new box
	current_scene.addCollisionObjects(collision_objects);

	moveit::planning_interface::MoveGroupInterface::Plan my_plan;
	const robot_state::JointModelGroup *joint_model_group =
  group.getCurrentState()->getJointModelGroup("arm");

  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<moveit_msgs::PickupAction> ac("pickup", true);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time

  ROS_INFO("Action server started, sending goal.");

	moveit_msgs::PickupGoal goal;
	goal.target_name = "grasping_object";
	goal.group_name = "arm";
	goal.group_name = "gripper";
	goal.possible_grasps.resize(1);





	/*
  // send a goal to the action
  actionlib_tutorials::FibonacciGoal goal;
  goal.order = 20;
  ac.sendGoal(goal);

  //wait for the action to return
  bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else
    ROS_INFO("Action did not finish before the time out.");
*/
  //exit
  return 0;
}
