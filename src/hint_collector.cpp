/**
* \file hint_collector.cpp
* \brief This file contains the implementation for the 'hint_collector' node.
* \author Muhammad Ali Haider Dar
* \version 1.0
* \date 12/08/2024
*
* \details
* 
* Services: <BR>
* /request_hint_collector
* /hint_loader_service
* 
* Publishers / Subscribers: <BR>
* /oracle_hint
*  
* Description :
*   This node provides the '/request_hint_collector' service, which collects and stores hints from the '/oracle_hint' topic. 
*   Once three hints are collected, it checks their consistency and loads them into the 'ARMOR' ontology knowledge base. It then starts the
*   ontology reasoner and verifies if the deduced hypothesis is complete and correct. If the hypothesis is inconsistent, incomplete, or
*   incorrect, it returns 'false'; otherwise, it returns 'true'.
*  
* 
* */

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <std_msgs/Float64.h>
#include <sstream>
#include <iostream>
#include <string>
#include "erl2/ErlOracle.h"
#include "erl2/HintLoader.h"
#include <std_srvs/Empty.h>
#include <erl2/HintCollector.h>
#include <cstdlib>
#include <unistd.h>


///< Global variable used for storing the first hint value.
erl2::ErlOracle hint1;

///< Global variable used for storing the second hint value.
erl2::ErlOracle hint2;

///< Global variable used for storing the third hint value.
erl2::ErlOracle hint3;

///< Client variable for the 'hint_loader_service' ROS service. 
ros::ServiceClient hint_loader_client;

///< Global variable to keep track of the total number of hints collected.
std::int64_t count = 0;

/**
* \brief Callback function for the '/oracle_hint' topic subscriber.
* \param msg The message of type 'erl2::ErlOracle'.
* \return None (void function).
*
* This function stores hints published to the '/oracle_hint' topic in global variables.
* 
*/
void oracle_hint_Callback(const erl2::ErlOracle::ConstPtr& msg)
{
	if(count == 0)
	{	
		count = 1;
	}
	
	if(count == 1)
	{
		hint1.ID = msg->ID;
		hint1.key = msg->key;
		hint1.value = msg->value;
	}
	
	else if(count == 2)
	{
		hint2.ID = msg->ID;
		hint2.key = msg->key;
		hint2.value = msg->value;
	}
	
	else if(count == 3)
	{
		hint3.ID = msg->ID;
		hint3.key = msg->key;
		hint3.value = msg->value;
	}
	
	ROS_INFO("count: %ld\n",count);
}

/**
* \brief Function to check the consistency of all collected hints.
* \param None
* \return Returns a boolean value indicating the result of the consistency check.
*
* This function verifies if all three hints have the same IDs and properly filled 'key' and 'value' fields. 
* Hints are considered inconsistent if these conditions are not met.
* 
*/
bool check_consistency()
{
	if(hint1.ID == hint2.ID && hint1.ID == hint3.ID)
	{
		if(hint1.key != "" && hint2.key != "" && hint3.key != "")
		{
			if(hint1.key != "-1" && hint2.key != "-1" && hint3.key != "-1")
			{
				if(hint1.value != "" && hint2.value != "" && hint3.value != "")
				{
					if(hint1.value != "-1" && hint2.value != "-1" && hint3.value != "-1")
					{ return true; }
					else
					{ return false; }			
				}
				else
				{ return false; }
			}
			else
			{ return false; }
		}
		else
		{ return false; }
	}
	else
	{ return false; }
}

/**
* \brief Function to load the hints into the ontology knowledge base.
* \param None
* \return Returns a boolean value; 'True' if the hints are successfully loaded into the knowledge base.
*
* This function uses the 'hint_loader_service' service to load the hints into the ontology knowledge base.
* 
*/
bool load_hint()
{
	
	erl2::HintLoader hint_loader_srv;
	hint_loader_srv.request.req.ID = hint1.ID;
	hint_loader_srv.request.req.key = hint1.key;
	hint_loader_srv.request.req.value = hint1.value;
	ROS_INFO("loading hint1.");
	if (hint_loader_client.call(hint_loader_srv))
	{
		if(hint_loader_srv.response.res)
		{
			ROS_INFO("Successfully loaded hint1.");
		}
	}
	else
	{
	  ROS_ERROR("Failed to call service request_hint_collector for hint1");
	  return false;
	}

	if(hint_loader_srv.response.res)
	{
		hint_loader_srv.request.req.ID = hint2.ID;
		hint_loader_srv.request.req.key = hint2.key;
		hint_loader_srv.request.req.value = hint2.value;
		ROS_INFO("loading hint2.");
		if (hint_loader_client.call(hint_loader_srv))
		{
			if(hint_loader_srv.response.res)
			{
				ROS_INFO("Successfully loaded hint2.");
			}
		}
		else
		{
		  ROS_ERROR("Failed to call service request_hint_collector for hint2");
		  return false;
		}	
	}

	
	if(hint_loader_srv.response.res)
	{
		hint_loader_srv.request.req.ID = hint3.ID;
		hint_loader_srv.request.req.key = hint3.key;
		hint_loader_srv.request.req.value = hint3.value;
		ROS_INFO("loading hint 3.");
		if (hint_loader_client.call(hint_loader_srv))
		{
			if(hint_loader_srv.response.res)
			{
				ROS_INFO("Successfully loaded hint 3.");
				return true;
			}
		}
		else
		{
		  ROS_ERROR("Failed to call service request_hint_collector for hint3");
		  return false;
		}	
	}
}

/**
* \brief Function to start the ontology reasoner.
* \param None
* \return Returns a boolean value; 'True' if the ARMOR reasoner starts successfully.
*
* This function uses the 'hint_loader_service' service to start the ARMOR reasoner.
* 
*/
bool start_reasoner()
{
	erl2::HintLoader check_correctness_srv;
	check_correctness_srv.request.req.ID = -2;
	check_correctness_srv.request.req.key = "REASON";
	check_correctness_srv.request.req.value = "";
	if (hint_loader_client.call(check_correctness_srv))
	{
		if(check_correctness_srv.response.res)
		{
			ROS_INFO("Successfully start the reasoner.");
			return true;
		}
	}
	else
	{
	  ROS_ERROR("Failed to call service request_hint_collector for starting the reasoner.");
	  return false;
	}
	
}

/**
* \brief Function to check the completeness of the hypothesis.
* \param None
* \return Returns a boolean value; 'True' if the deduced hypothesis is complete.
*
* This function determines if the hypothesis, deduced from the previously loaded hints, is complete.
* 
*/
bool check_completeness()
{
	erl2::HintLoader check_completeness_srv;
	check_completeness_srv.request.req.ID = -11;
	check_completeness_srv.request.req.key ="COMPLETED";
	check_completeness_srv.request.req.value = "";
	if (hint_loader_client.call(check_completeness_srv))
	{
		if(check_completeness_srv.response.res)
		{
			//ROS_INFO("Successfully start the reasoner.");
			return true;
		}
	}
	else
	{
	  ROS_ERROR("Failed to call service request_hint_collector for starting the reasoner");
	  return false;
	}
	
}

/**
* \brief Function to check the correctness of the hypothesis based on previously collected hints.
* \param None
* \return Returns a boolean value.
*
* This function checks if the deduced hypothesis is correct by comparing its ID with the correct hypothesis ID provided by the 'oracle_solution' service.
* 
*/
bool check_correctness()
{
	ROS_INFO("checking correctness.");
	erl2::HintLoader check_correctness_srv;
	check_correctness_srv.request.req.ID = -2;
	check_correctness_srv.request.req.key = "CORRECTNESS";
	check_correctness_srv.request.req.value = "";
	if (hint_loader_client.call(check_correctness_srv))
	{
		if(check_correctness_srv.response.res)
		{
			return true;
		}
	}
	else
	{
	  ROS_ERROR("Failed to call service request_hint_collector for starting the reasoner");
	  return false;
	}
}

/**
* \brief Function to generate a hint statement.
* \param None
* \return Returns a string containing the hint statement.
*
* This function creates a hint statement by concatenating the values of previously collected hints.
* 
*/
std::string getHintStatement()
{
	std::string who_;
	std::string what_;
	std::string where_;

	// hint1
	if(hint1.key == "who")
		who_ = hint1.value;
	else if(hint1.key == "what")
		what_ = hint1.value;
	else if(hint1.key == "where")
		where_ = hint1.value;

	// hint2	
	if(hint2.key == "who")
		who_ = hint2.value;
	else if(hint2.key == "what")
		what_ = hint2.value;
	else if(hint2.key == "where")
		where_ = hint2.value;

	// hint3
	if(hint3.key == "who")
		who_ = hint3.value;
	else if(hint3.key == "what")
		what_ = hint3.value;
	else if(hint3.key == "where")
		where_ = hint3.value;
	
	std::string hintStatement = who_ + std::string(" with the ") + what_ + std::string(" in the ") + where_;
	ROS_INFO("Hint Statement %s",hintStatement.c_str());
	return hintStatement;
}

/**
* \brief Callback function for the 'request_hint_collector' service.
* \param req Request argument of the 'request_hint_collector' service, of type erl2::HintCollector::Request.
* \param res Response argument of the 'request_hint_collector' service, of type erl2::HintCollector::Response.
* \return Returns a boolean value.
*
* This function responds to client requests by storing the hints. If three hints are successfully collected, 
* it checks their consistency, completeness, and correctness using appropriate functions.
* 
*/
bool collect_hint(erl2::HintCollector::Request  &req, erl2::HintCollector::Response &res)
{
	ROS_INFO("Got the request %s", req.req.c_str());
	if(req.req == "collect")
	{	
		if(count == 1)
		{	
			if(hint1.ID >= 0)
			{
				ROS_INFO("Collected the Hint1: ID: %s, key: %s, value: %s",(std::to_string(hint1.ID)).c_str(), (hint1.key).c_str(),(hint1.value).c_str());
				count = count%3;
				count++;
			}
			else
			{ ROS_INFO("Hint1 array is empty."); }
		}	
		else if(count == 2)
		{
			if(hint2.ID >= 0)
			{
				ROS_INFO("Collected the Hint2: ID: %s, key: %s, value: %s", (std::to_string(hint2.ID)).c_str(), (hint2.key).c_str(),(hint2.value).c_str());
				count = count%3;
				count++;
			}
			else
			{ ROS_INFO("Hint2 array is empty."); }
		}	
		else if(count == 3)
		{
			if(hint3.ID >= 0)
			{
				ROS_INFO("Collected the Hint3: ID: %s, key: %s, value: %s",(std::to_string(hint3.ID)).c_str(), (hint3.key).c_str(),(hint3.value).c_str());
				if(check_consistency())
				{
					ROS_INFO("Hint is ready to be loaded.");
					if(load_hint())
					{
						if(start_reasoner())
						{
							if(check_completeness())
							{
								ROS_INFO("Complete hypothesis is found. Now checking correctness."); 
								if(check_correctness() == true)
						        	{  
									ROS_INFO("Hints are correct.. hurray.."); 	 
									res.hintStatement = getHintStatement();
									res.result = true;
									return true;
								}									
								else
								{ 
									ROS_INFO("Hints are not correct. Lets go again.");
									count = count%3;
									count++;
									res.result = false;
									return false; 
								}
							}
							else 
							{ 
								ROS_INFO("Hints are not complete. Lets go again."); 
								count = count%3;
								count++;
								res.result = false;
								return false; 
							}
						}
						else 
						{ 
							ROS_INFO("There was an error in starting the reasoner");
							return false; 
						}	
					}
					else 
					{ 
						ROS_INFO("There was an error in loading the hints");
						res.result = false; 
						return false; 
					}
				}
				else 
				{ 
					ROS_INFO("hints are not consistent. Moving on...");
					count = count%3;
					count++;
					res.result = false;
					return false; 
				}
				count = count%3;
				count++;
			}
			else
			{ 
				ROS_INFO("Hint3 array is empty.");
			}	
		}
		res.hintStatement = "";
		res.result = true;
		return true; 
	}
}

/**
* \brief Main function for the 'hint_collector' node.
* \param argc Integer argument.
* \param argv Double pointer to a string argument.
* \return Always returns 0, as this function does not fail.
*
* This function initializes the ROS node, sets up the server for the 'request_hint_collector' service, 
* and creates a client for the 'hint_loader_service' service.
* 
*/
int main(int argc, char **argv)
{
  ros::init(argc, argv, "hint_collector");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/oracle_hint", 10, oracle_hint_Callback);
  ros::ServiceServer service = n.advertiseService("request_hint_collector", collect_hint);
  hint_loader_client = n.serviceClient<erl2::HintLoader>("hint_loader_service");
  std::cout << "'request_hint_collector' service is live.." << std::endl;
  ros::spin();
  return 0;
}
