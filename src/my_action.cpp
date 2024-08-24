/**
* \file my_action.cpp
* \brief This file contains the implementation for the 'my_action' node.
* \author Muhammad Ali Haider Dar
* \version 1.0
* \date 12/08/2024
*
* \details
* 
* Services : <BR>
* /request_hint_collector
* /request_move_arm
* /request_set_orientation
* /rosplan_plan_dispatcher/cancel_dispatch
* /rosplan_knowledge_base/update_array
* /request_replan
* 
* Action Clients / Services : <BR>
* 	/reaching_goal
*  
* Description :
*
* This node implements the action client that tasks the robot in the Gazebo simulation to visit all waypoints (wp1, wp2, wp3, wp4). When the robot reaches a waypoint, the node
* calls the 'request_set_orientation' service to adjust the robots orientation for optimal arm movement. Once oriented, the robot is instructed to move its arm 
* using the 'request_move_arm' service and collect hints. If hints are successfully collected, the service returns 'true', and the robot proceeds to the next waypoint.
*/

#include "erl2/my_action.h"
#include "erl2/ErlOracle.h"
#include <erl2/HintCollector.h>
#include <erl2/SetOrien.h>
#include <erl2/MoveArm.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Float64.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include "std_msgs/String.h"
#include <motion_plan/PlanningAction.h>
#include <iostream>
#include <string>
#include "ros/ros.h"
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>


namespace KCL_rosplan 
{	
	/**
	* \brief Constructor for the MyActionInterface class within the KCL_rosplan namespace. This class inherits from the RPActionInterface class.
	* \param nh A ROS node handle.
	* \return None
	*
	* The constructor initializes another ROS node handle within the KCL_rosplan namespace and the MyActionInterface class.
	* 
	*/	
	MyActionInterface::MyActionInterface(ros::NodeHandle &nh)
	{
		// initialization
		ros::NodeHandle n = nh;
	}

	/**
	* \brief Callback function for the MyActionInterface class.
	* \param msg The task argument of type 'rosplan_dispatch_msgs::ActionDispatch'.
	* \return Returns a boolean value based on the outcome of the task assigned to the action server.
	*
	* This function serves as a callback for the 'goto_waypoint' action in the PDDL domain file. During plan dispatch, 
	* it is triggered by the 'goto_waypoint' action. It includes the low-level control logic for the 'goto_waypoint' action,
	* which moves the robot in the Gazebo simulation from one waypoint to another using an action service. Once the robot
	* reaches the desired waypoint, it calls the 'request_set_orientation' and 'request_move_arm' services to collect hints.
	* 
	*/
	bool MyActionInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg)
  	{	
		std::cout << " Going from " << msg->parameters[1].value << " to " << msg->parameters[2].value << std::endl;
		
		//Implementing action client
		actionlib::SimpleActionClient<motion_plan::PlanningAction> ac("reaching_goal", true);
		
		motion_plan::PlanningGoal goal;
		ac.waitForServer();	
		
		std::string hintStatement;
		
		// Assigning coordinates to wp1
		if(msg->parameters[2].value == "wp1")
		{
			goal.target_pose.pose.position.x = -2.1;
			goal.target_pose.pose.position.y = 0.0;			
			goal.target_pose.pose.orientation.w = 0.0;
		}
		
		// Assigning coordinates to wp2
		else if (msg->parameters[2].value == "wp2")
		{			
			goal.target_pose.pose.position.x = 2.1;
			goal.target_pose.pose.position.y = 0.0;
			goal.target_pose.pose.orientation.w = 0.0;
		}
		
		// Assigning coordinates to wp3
		else if (msg->parameters[2].value == "wp3")
		{ 
			goal.target_pose.pose.position.x = 0.0;
			goal.target_pose.pose.position.y = -2.1;
			goal.target_pose.pose.orientation.w = 0.0;
		}
		
		// Assigning coordinates to wp4
		else if (msg->parameters[2].value == "wp4")
		{
			goal.target_pose.pose.position.x = 0.0;
			goal.target_pose.pose.position.y = 2.1;
			goal.target_pose.pose.orientation.w = 0.0;
		}
		
		ac.sendGoal(goal);

		// Waiting for action client response
		bool finished_before_timeout = ac.waitForResult();
		
		if (finished_before_timeout) 
		{
			actionlib::SimpleClientGoalState state = ac.getState();
        		ROS_INFO("KCL: (%s) action finished: %s", params.name.c_str(), state.toString().c_str());

			// If action client return success
			if(state == actionlib::SimpleClientGoalState::SUCCEEDED) 
			{
				if (msg->parameters[2].value == "wp1" || msg->parameters[2].value == "wp2" 
					 || msg->parameters[2].value == "wp3" || msg->parameters[2].value == "wp4")
				{
					ROS_INFO("Action (%s) performed: completed!", msg->name.c_str());
					ROS_INFO("Now collecting hint...");
						
					// Initializing service clients that will be use to collect the hints
					ros::ServiceClient hint_client = n.serviceClient<erl2::HintCollector>("request_hint_collector");
					ros::ServiceClient move_arm_client = n.serviceClient<erl2::MoveArm>("request_move_arm");
					ros::ServiceClient set_orien_client = n.serviceClient<erl2::SetOrien>("request_set_orientation");
					
						
					// initializing 'request_hint_collector' service request msg
					erl2::HintCollector srv_hintcollector;
					srv_hintcollector.request.req = "collect";
					
					// initializing 'request_move_arm' service request msg
					erl2::MoveArm srv_movearm;
					srv_movearm.request.wp = msg->parameters[2].value;

					// initializing 'request_set_orientation' service request msg
					erl2::SetOrien srv_set_orien;
					srv_set_orien.request.req = "set";
					
					geometry_msgs::Pose pose1;  

						
					if(msg->parameters[2].value == "wp1")
					{
						srv_set_orien.request.goal = -1.50;
						if(set_orien_client.call(srv_set_orien))
						{
							pose1.orientation.w = 1.0;
							pose1.orientation.x = 0.0;
							pose1.orientation.y = 0.0;
							pose1.orientation.z = 0.0;
							pose1.position.x =  0.0;
							pose1.position.y =  -1.0;
							pose1.position.z = 1.25;
						}
					}
					else if(msg->parameters[2].value == "wp2")
					{
						srv_set_orien.request.goal = 1.50;
						if(set_orien_client.call(srv_set_orien))
						{
							pose1.orientation.w = 1.0;
							pose1.orientation.x = 0.0;
							pose1.orientation.y = 0.0;
							pose1.orientation.z = 0.0;
							pose1.position.x =  0.0;
							pose1.position.y =  -1.0;
							pose1.position.z = 1.25;
						}						
					}
					else if(msg->parameters[2].value == "wp3")
					{
						srv_set_orien.request.goal = -3.0;
						if(set_orien_client.call(srv_set_orien))
						{
							pose1.orientation.w = 1.0;
							pose1.orientation.x = 0.0;
							pose1.orientation.y = 0.0;
							pose1.orientation.z = 0.0;
							pose1.position.x =  0.0;
							pose1.position.y =  -3.0;
							pose1.position.z = 1.25;
						}						
					}
					else if(msg->parameters[2].value == "wp4")
					{
						srv_set_orien.request.goal = -3.0;
						if(set_orien_client.call(srv_set_orien))
						{
							pose1.orientation.w = 1.0;
							pose1.orientation.x = 0.0;
							pose1.orientation.y = 0.0;
							pose1.orientation.z = 0.0;
							pose1.position.x =  0.0;
							pose1.position.y =  3.0;
							pose1.position.z = 1.25;					
						}
					}						
					srv_movearm.request.pose = pose1;	
						
					// calling the service request_move_arm
					if(move_arm_client.call(srv_movearm))
					{
						pose1.position.z = 0.75;
						srv_movearm.request.pose = pose1;	
						
						// calling the 'request_move_arm' service again for z = 0.75
						if(move_arm_client.call(srv_movearm))
						{
							// calling the 'request_hint_collector' service 
							if(hint_client.call(srv_hintcollector))
							{
								if(srv_hintcollector.response.result)
								{
									ROS_INFO("Hints collected successfully..");
									hintStatement = srv_hintcollector.response.hintStatement;
									
									if(hintStatement != "")
										//Making wp1,wp2,wp3,wp4 as visited so the robot goes wp0 directly.
										updateKB("set_wp_visited");
									return true;
								}
							}
							else
							{
								ROS_INFO("Hints not collected successfully..");	
								//Making wp1,wp2,wp3,wp4 as not visited so the robot goes these waypoints again and collect new hints
								updateKB("replan");
								return 1;
							}
						}
					}
					else
					{
						ROS_ERROR("Failed to call service request_move_arm");
						return 1;
					}
				}
				if(msg->parameters[2].value == "wp0")
				{
					ROS_INFO("Robot reached wp0.");
					ROS_INFO("Hint Statement %s",hintStatement.c_str());
					ROS_INFO("Game Over.");
				}				
			}
		}
   		else
    		{
			// timed out (failed)
			ac.cancelAllGoals();
			ROS_INFO("KCL: (%s) action timed out", params.name.c_str());
			return false;
    		}
	}

	/**
	* \brief Function to update the knowledge in the ROSPlan Knowledge Base (KB).
	* \param req A string argument indicating the type of update needed in the KB.
	* \return None (void function).
	*
	* This function updates the knowledge in the ROSPlan KB under two conditions. First, if the robot collects hints that are inconsistent, incomplete, or incorrect, 
	* the knowledge about the visited waypoints is removed from the KB. This allows the robot to revisit all waypoints after replanning to gather new hints. 
	* Second, if the robot collects hints that are consistent, complete, and correct, all waypoints are marked as visited, so the robot can proceed directly to wp0 
	* to display the hint statement.
	* 
	*/	
	void MyActionInterface::updateKB(const std::string& req)
	{
		// Initializing services
		ros::ServiceClient cancel_dispatch_client = n.serviceClient<std_srvs::Empty>("/rosplan_plan_dispatcher/cancel_dispatch");
		ros::ServiceClient kb_updade_client = n.serviceClient<rosplan_knowledge_msgs::KnowledgeUpdateServiceArray>("/rosplan_knowledge_base/update_array");
		ros::ServiceClient replan_client = n.serviceClient<std_srvs::Empty>("/request_replan");
		
		std::string waypoint[4] = {"wp1","wp2","wp3","wp4"};
		std_srvs::Empty empty;
		
		if(req == "replan")
		{
			// call services to update KB regarding goals. 
			ROS_INFO("Updating KB to achieve goals again.");
			
			ROS_INFO("Cancelling Dispatcher..");
			cancel_dispatch_client.call(empty);
			//Remove already achieved goals.
			
			// update knowledge base
			rosplan_knowledge_msgs::KnowledgeUpdateServiceArray updatemsgSrv1;		
			
			for(int i=0; i<4; i++)
			{
				rosplan_knowledge_msgs::KnowledgeItem item1;
				item1.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
				item1.attribute_name = "visited";
				item1.values.clear();
				diagnostic_msgs::KeyValue pair1;		
				pair1.key = "wp";
				pair1.value = waypoint[i];
				item1.values.push_back(pair1);
					
				updatemsgSrv1.request.knowledge.push_back(item1);
				updatemsgSrv1.request.update_type.push_back(rosplan_knowledge_msgs::KnowledgeUpdateService::Request::REMOVE_KNOWLEDGE);
			}
			
			if(updatemsgSrv1.request.knowledge.size()>0 && !kb_updade_client.call(updatemsgSrv1))
					ROS_INFO("KCL: failed to update PDDL model in knowledge base");
			replan_client.call(empty);		
		}
		
		else if(req == "set_wp_visited")
		{
			// call services to update KB regarding goals. 
			ROS_INFO("Updating KB to achieve goals again.");
			
			ROS_INFO("Cancelling Dispatcher..");
			cancel_dispatch_client.call(empty);
			//Remove already achieved goals.
			
			// update knowledge base
			rosplan_knowledge_msgs::KnowledgeUpdateServiceArray updatemsgSrv2;		
			
			for(int i=0; i<4; i++)
			{
				rosplan_knowledge_msgs::KnowledgeItem item2;
				item2.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
				item2.attribute_name = "visited";
				item2.values.clear();
				diagnostic_msgs::KeyValue pair2;		
				pair2.key = "wp";
				pair2.value = waypoint[i];
				item2.values.push_back(pair2);
					
				updatemsgSrv2.request.knowledge.push_back(item2);
				updatemsgSrv2.request.update_type.push_back(rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE );
			}
			
			if(updatemsgSrv2.request.knowledge.size()>0 && !kb_updade_client.call(updatemsgSrv2))
				ROS_INFO("KCL: failed to update PDDL model in knowledge base");				
				
			replan_client.call(empty);	
		}
	}
} // close namespace


/**
* \brief The main function of the 'my_action' node.
* \param argc Integer argument.
* \param argv Double pointer to a string argument.
* \return Always returns 0, as this function cannot fail.
*
* This function initializes the ROS node and the 'MyActionInterface' class under the KCL_rosplan namespace.
* 
*/
int main(int argc, char **argv) 
{
	ros::init(argc, argv, "erl2_rosplan_action", ros::init_options::AnonymousName);
	ros::NodeHandle nh("~");
	KCL_rosplan::MyActionInterface my_aci(nh);
	my_aci.runActionInterface();
	
	return 0;
}
