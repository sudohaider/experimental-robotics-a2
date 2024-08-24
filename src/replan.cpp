/**
* \file replan.cpp
* \brief This file contains the implementation for the 'replan' node.
* \author Muhammad Ali Haider Dar
* \version 1.0
* \date 12/08/2024
*
* \details
* 
* Services : <BR>
* /request_replan
* 
* Publishers / Subscribers : <BR>
* /replan
*  
* Description :
* 
* This node provides the '/request_replan' service, which signals the 'replan_sub' node to initiate the replanning process by publishing
* the string "replan" to the 'replan' topic.
*/

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <std_msgs/Float64.h>
#include <sstream>
#include <iostream>
#include <string>
#include <std_srvs/Empty.h>
#include <cstdlib>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>

///< Initializing ROS Publisher 'replan_pub_' variable for '\replan' topic.
ros::Publisher replan_pub_;


/**
* \brief Callback function for the '/request_replan' service.cc
* \param req Request argument for the '/request_replan' service, of type std_srvs::Empty::Request.
* \param res Response argument for the '/request_replan' service, of type std_srvs::Empty::Response.
* \return Returns a boolean value.
*
* This function handles client requests by publishing the string "replan" to the 'replan' topic, signaling the corresponding subscriber
* to begin the replanning process by executing the 'rosplan_start.sh' script.
* 
*/
bool replan_srv_callback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
	std_msgs::String msg;
	msg.data = "replan";
	replan_pub_.publish(msg);
	return true;
}

/**
* \brief The main function of the 'replan' node.
* \param argc Integer argument.
* \param argv Double pointer to a string argument.
* \return Always returns 0, as this function cannot fail.
*
* This function initializes the ROS node, sets up the server for the '/request_replan' service, and creates a publisher for the 'replan' topic.
*/
int main(int argc, char **argv)
{
	ros::init(argc, argv, "replan");
	ros::NodeHandle n;
	ROS_INFO("'request_replan' service is live...");
	ros::ServiceServer service = n.advertiseService("/request_replan", replan_srv_callback);
	replan_pub_ = n.advertise<std_msgs::String>("replan", 100);
	ros::spin();
	return 0;
}
