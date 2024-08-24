/**
* \file move_arm.cpp
* \brief This files contains code for the 'move_arm' node.
* \author Muhammad Ali Haider Dar
* \version 1.0
* \date 12/08/2024
*
* \details
* 
* Services : <BR>
* /request_move_arm
*  
* Description :
*
* This node plans and executes the robot's arm movements using the ROS MoveIt library.
* It sets up a service through which a client can request the robot's arm end-effector
* to move to a specified position in the Gazebo simulation.
* 
*/

#include <ros/ros.h>
// MoveIt
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <erl2/MoveArm.h>
#include "std_msgs/String.h"
#include <cstdlib>
#include <unistd.h>

/**
* \brief Callback function for the '/request_move_arm' service.
* \param req Request argument for the '/request_move_arm' service, of type erl2::MoveArm::Request.
* \param res Response argument for the '/request_move_arm' service, of type erl2::MoveArm::Response.
* \return Returns a boolean value.
*
* This function handles client requests by planning and executing the robot's arm motion using the ROS MoveIt library.
* 
*/
bool move_arm(erl2::MoveArm::Request  &req, erl2::MoveArm::Response &res)
{
	ros::AsyncSpinner spinner(1);
	spinner.start();
  
	robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
	const moveit::core::RobotModelPtr kinematic_model = robot_model_loader.getModel();
	ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());
	
	moveit::core::RobotStatePtr kinematic_state(new moveit::core::RobotState(kinematic_model));
	kinematic_state->setToDefaultValues();
	const moveit::core::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("arm");
	moveit::planning_interface::MoveGroupInterface group("arm");
	const std::vector<std::string> &joint_names = joint_model_group->getVariableNames();
  
	ROS_INFO("req.wp == %s", req.wp.c_str());
	geometry_msgs::Pose pose1;
	pose1 = req.pose;
	  
	ROS_INFO("Setting (x,y,z) == (%f,%f,%f)", pose1.position.x,pose1.position.y,pose1.position.z);
	  
	group.setStartStateToCurrentState();
	group.setApproximateJointValueTarget(pose1,"cluedo_link");
	std::vector<double> joint_values;
	double timeout = 0.1;
  
	//ROS_INFO("joint_model_group == %s", joint_model_group->.c_str());
  
	bool found_ik = kinematic_state->setFromIK(joint_model_group, pose1, "cluedo_link", 10, timeout);

	// Now, we can print out the IK solution (if found):
	if (found_ik)
  	{
    		kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
	    	for (std::size_t i = 0; i < joint_names.size(); ++i)
	    	{
	      		ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
	    	}
  	}
	else
	{
		ROS_INFO("Did not find IK solution");
	}

	group.setJointValueTarget(joint_values);
	group.setStartStateToCurrentState();
	group.setGoalOrientationTolerance(0.01);
	group.setGoalPositionTolerance(0.01);

	// Plan and execute
	moveit::planning_interface::MoveGroupInterface::Plan my_plan;
	group.plan(my_plan);
	group.execute(my_plan);
    
	std::cout << "Position 1 -> IK + setJointValue" << std::endl;
	sleep(10.0);

	// standing straight

	pose1.position.y = 0.0;
	pose1.position.z = 1.6;
  
	ROS_INFO("Setting (x,y,z) == (%f,%f,%f)", pose1.position.x,pose1.position.y,pose1.position.z);
  
  	group.setStartStateToCurrentState();
  	group.setApproximateJointValueTarget(pose1,"cluedo_link");
    
  	found_ik = kinematic_state->setFromIK(joint_model_group, pose1, "cluedo_link", 10, timeout);

  	if (found_ik)
  	{
		kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
		for (std::size_t i = 0; i < joint_names.size(); ++i)
		{
			ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
		}
	}
	else
	{
		ROS_INFO("Did not find IK solution");
	}

	group.setJointValueTarget(joint_values);
	group.setStartStateToCurrentState();
	group.setGoalOrientationTolerance(0.01);
	group.setGoalPositionTolerance(0.01);

	group.plan(my_plan);
	group.execute(my_plan);
    
	std::cout << "Position 1 -> IK + setJointValue" << std::endl;
	sleep(10.0);  
  	
	res.res = true;
	return true;    
}

/**
* \brief The main function of the 'move_arm' node.
* \param argc Integer argument.
* \param argv Double pointer to a string argument.
* \return Always returns 0, as this function cannot fail.
*
* This function initializes the ROS node handle and the 'request_move_arm' service.
* 
*/
int main(int argc, char** argv)
{
	ros::init(argc, argv, "robot_model_and_robot_state_tutorial");
	ros::NodeHandle nh;
	ros::ServiceServer service = nh.advertiseService("request_move_arm", move_arm);
	std::cout << "'request_move_arm' service is live.." << std::endl;
	ros::AsyncSpinner spinner(1);
	spinner.start();
	ros::waitForShutdown();
	return(0);
}
