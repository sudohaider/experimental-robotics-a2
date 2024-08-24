/**
* \file replan_sub.cpp
* \brief This file contains the implementation for the 'replan_sub' node.
* \author Muhammad Ali Haider Dar
* \version 1.0
* \date 12/08/2024
*
* \details
* 
* Publishers / Subscribers : <BR>
* /replan
*  
* Description :
*
* This node initializes a subscriber for the '/replan' topic. The callback function associated with this subscriber 
* executes the 'rosplan_start.sh' script to initiate the replanning process.
*/

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <stdio.h>
#include <stdlib.h>


/**
* \brief Callback function for the 'replan' topic subscriber.
* \param req Argument of type std_msgs::String, representing the message received on the 'replan' topic.
* \return None (void function).
*
* This function initiates the replanning process by executing the 'rosplan_start.sh' script.
*/
void replan_callback(const std_msgs::String::ConstPtr& msg)
{
	if(msg->data == "replan")
	{
		ROS_INFO("Starting replanning");
		system("/root/ros_ws/src/erl2/scripts/rosplan_start.sh"); 
	}
}

/**
* \brief The main function of the 'replan_sub' node.
* \param argc Integer argument.
* \param argv Double pointer to a string argument.
* \return Always returns 0, as this function cannot fail.
*
* This function initializes the ROS node and sets up the subscriber for the 'replan' topic.
*/
int main(int argc, char **argv)
{
  ros::init(argc, argv, "replan_sub");
  ros::NodeHandle n;
  ROS_INFO("'replan_sub' node is live...");
  ros::Subscriber sub = n.subscribe("replan", 100, replan_callback);
  ros::spin();
  return 0;
}
