#! /usr/bin/env python2

## @package erl2
# \file hint_loader.py
# \brief This file contains code for 'hint_loader' node.
# \author Muhammad Ali Haider Dar
# \version 1.0
# \date 12/08/2024
#
# \details
#
# Service : <BR>
# /hint_loader_service
# /armor_interface_srv
# /oracle_solution
#  
# This node listens for service requests from the 'hint_collector' node via the 'hint_loader_service'. Upon receiving a request, it
# loads the hint into the ARMOR reasoner, activates the reasoner to deduce a hypothesis based on the previously loaded hints, and
# queries the ARMOR reasoner for a list of 'COMPLETE' hypotheses. If the newly deduced hypothesis is marked 'COMPLETE', it then
# verifies its 'CORRECTNESS'.
#

import rospy
import time
from erl2.srv import Oracle, OracleRequest, OracleResponse
from erl2.msg import ErlOracle
from erl2.srv import HintLoader, HintLoaderResponse, HintLoaderRequest
from armor_msgs.srv import ArmorDirective,ArmorDirectiveRequest


## Assigning 'None' to the global variable 'oracle_client_' for the '/oracle_solution' service client.
oracle_client_ = None

## Assigning 'None' to the global variable 'armor_client_' for the '/armor_interface_srv' service client.
armor_client_ = None

## Assigning 'None' to the global variable 'armor_req_' for the '/armor_interface_srv' service request.
armor_req_ = None

## Initializing the global variable 'count_' to '0'.
count_ = 0

## Initializing the global variable 'prev_comp_hypo_' to '0'.
prev_comp_hypo_ = 0

## Setting the global variable 'who_' to 'none'.
who_ = "none"

## Setting the global variable 'where_' to 'none'.
where_ = "none"

## Setting the global variable 'what_' to 'none'.
what_ = "none"

## Setting the global variable 'ID_' to '-1'.
ID_ = -1


##
# \brief This is the 'clbk_oracle_service' function for the oracle node.
# 
# \return Bool
#
# This function serves as a callback for the '/oracle_service' node. It waits for a service request from the 'motion_controller' node.
# Depending on the type of request received, it loads the hint into the ARMOR reasoner, starts the reasoner to deduce a hypothesis based
# on the previously loaded hints, and queries the ARMOR reasoner for a list of 'COMPLETE' hypotheses. If the hypothesis is 'CONSISTENT',
# it returns 'True'; otherwise, it returns 'False'. If the hypothesis is 'CONSISTENT', the user can also request the node to check if the
# hypothesis is 'CORRECT'. A hypothesis is considered correct if it is 'CONSISTENT' and belongs to a predefined list of correct hypotheses
# within this node. If the hypothesis is correct, the node returns 'True'; otherwise, it returns 'False'.
#
def clbk_oracle_service(msg):
	global count_
	global armor_req_
	global armor_res_
	global prev_comp_hypo_
	global who_ 
	global where_
	global what_ 
	global ID_

	if(count_ == 0):

		# Making sure that this section run only once. 
		count_ += 1	
		
		# Loading the ontology file in the reasoner.
		armor_req_ = ArmorDirectiveRequest()
		armor_req_.armor_request.client_name = 'tutorial'
		armor_req_.armor_request.reference_name = 'ontoTest'
		armor_req_.armor_request.command = 'LOAD'
		armor_req_.armor_request.primary_command_spec = 'FILE'
		armor_req_.armor_request.secondary_command_spec = ''
		armor_req_.armor_request.args = ['/root/Desktop/cluedo_ontology.owl', 'http://www.emarolab.it/cluedo-ontology', 'true', 'PELLET', 'true']
		print(armor_req_)
		armor_res_ = armor_client_(armor_req_)
		print(armor_res_)
		
	if(msg.req.key == "who" and armor_res_.armor_response.success == True):
		
		# loading the hint in the reasoner.
		print("msg->key :", msg.req.key)
		array1 = [msg.req.key,str(msg.req.ID),msg.req.value]
		
		who_ = msg.req.value
		ID_ = msg.req.ID
		
		armor_req_.armor_request.client_name = 'tutorial'
		armor_req_.armor_request.reference_name = 'ontoTest'
		armor_req_.armor_request.command = 'ADD'
		armor_req_.armor_request.primary_command_spec = 'OBJECTPROP'
		armor_req_.armor_request.secondary_command_spec = 'IND'
		armor_req_.armor_request.args = array1
		print(armor_req_)
		armor_res_ = armor_client_(armor_req_)
		print(armor_res_)
		
		time.sleep(1)
		
		armor_req_.armor_request.client_name = 'tutorial'
		armor_req_.armor_request.reference_name = 'ontoTest'
		armor_req_.armor_request.command = 'ADD'
		armor_req_.armor_request.primary_command_spec = 'IND'
		armor_req_.armor_request.secondary_command_spec = 'CLASS'
		armor_req_.armor_request.args = [msg.req.value,'PERSON']
		print(armor_req_)
		armor_res_ = armor_client_(armor_req_)
		print(armor_res_)
		
		if(armor_res_.armor_response.success == True):
			return HintLoaderResponse(True)
		
	elif(msg.req.key == "what" and armor_res_.armor_response.success == True):
		
		# loading the hint in the reasoner.
		print("msg.command :", msg.req.key)
		
		what_ = msg.req.value
		array2 = [msg.req.key,str(msg.req.ID),msg.req.value]
		
		armor_req_.armor_request.client_name = 'tutorial'
		armor_req_.armor_request.reference_name = 'ontoTest'
		armor_req_.armor_request.command = 'ADD'
		armor_req_.armor_request.primary_command_spec = 'OBJECTPROP'
		armor_req_.armor_request.secondary_command_spec = 'IND'
		armor_req_.armor_request.args = array2
		print(armor_req_)
		armor_res_ = armor_client_(armor_req_)
		print(armor_res_)
		
		time.sleep(1)
		
		armor_req_.armor_request.client_name = 'tutorial'
		armor_req_.armor_request.reference_name = 'ontoTest'
		armor_req_.armor_request.command = 'ADD'
		armor_req_.armor_request.primary_command_spec = 'IND'
		armor_req_.armor_request.secondary_command_spec = 'CLASS'
		armor_req_.armor_request.args = [msg.req.value,'WEAPON']
		print(armor_req_)
		armor_res_ = armor_client_(armor_req_)
		print(armor_res_)
		
		if(armor_res_.armor_response.success == True):
			return HintLoaderResponse(True)
	
	elif(msg.req.key == "where" and armor_res_.armor_response.success == True):
		
		# loading the hint in the reasoner.
		print("msg.command :", msg.req.key)
	
		where_ = msg.req.value
		array3 = [msg.req.key,str(msg.req.ID),msg.req.value]
		
		
		armor_req_.armor_request.client_name = 'tutorial'
		armor_req_.armor_request.reference_name = 'ontoTest'
		armor_req_.armor_request.command = 'ADD'
		armor_req_.armor_request.primary_command_spec = 'OBJECTPROP'
		armor_req_.armor_request.secondary_command_spec = 'IND'
		armor_req_.armor_request.args = array3
		print(armor_req_)
		armor_res_ = armor_client_(armor_req_)
		print(armor_res_)
		
		time.sleep(1)
		
		armor_req_.armor_request.client_name = 'tutorial'
		armor_req_.armor_request.reference_name = 'ontoTest'
		armor_req_.armor_request.command = 'ADD'
		armor_req_.armor_request.primary_command_spec = 'IND'
		armor_req_.armor_request.secondary_command_spec = 'CLASS'
		armor_req_.armor_request.args = [msg.req.value,'PLACE']
		print(armor_req_)
		armor_res_ = armor_client_(armor_req_)
		print(armor_res_)
		
		if(armor_res_.armor_response.success == True):
			return HintLoaderResponse(True)
		
	elif(msg.req.key == "REASON" and armor_res_.armor_response.success == True):

		# Starting the reasoner
		print("msg.key :", msg.req.key)
		
		armor_req_.armor_request.client_name = 'tutorial'
		armor_req_.armor_request.reference_name = 'ontoTest'
		armor_req_.armor_request.command = 'REASON'
		armor_req_.armor_request.primary_command_spec = ''
		armor_req_.armor_request.secondary_command_spec = ''
		armor_req_.armor_request.args = []
		print(armor_req_)
		armor_res_ = armor_client_(armor_req_)
		print(armor_res_)
		
		if(armor_res_.armor_response.success == True):
			return HintLoaderResponse(True)
		
	elif(msg.req.key == "COMPLETED" and armor_res_.armor_response.success == True):	
		# Starting the reasoner
		print("msg.key :", msg.req.key)
		
		armor_req_.armor_request.client_name = 'tutorial'
		armor_req_.armor_request.reference_name = 'ontoTest'
		armor_req_.armor_request.command = 'QUERY'
		armor_req_.armor_request.primary_command_spec = 'IND'
		armor_req_.armor_request.secondary_command_spec = 'CLASS'
		armor_req_.armor_request.args = ['COMPLETED']
		print(armor_req_)
		armor_res_ = armor_client_(armor_req_)
		print(armor_res_)
		
		if(armor_res_.armor_response.success == True):
					
			new_comp_hypo =  len(armor_res_.armor_response.queried_objects)
			
			print("NEW COMPLETE HYPOTHESIS: ", new_comp_hypo)
			print("PREVIOUS COMPLETE HYPOTHESIS: ", prev_comp_hypo_)
			
			if(new_comp_hypo > prev_comp_hypo_):
				prev_comp_hypo_ = 	new_comp_hypo		
				return HintLoaderResponse(False)
			else:
				return HintLoaderResponse(True)
	
	elif(msg.req.key == "CORRECTNESS" and armor_res_.armor_response.success == True):	
		
		print("msg.key :", msg.req.key)
		
		#calling the 'oracle_solution' service for correct solution. 
		correct_solution_id = oracle_client_()
		print("correct_solution_id = ", correct_solution_id.ID)
		print("ID_ =", ID_)
		
		if(correct_solution_id.ID == ID_):
			return HintLoaderResponse(True)
		else:
			return HintLoaderResponse(False)


##
# \brief This is the 'main' function of the oracle node.
# 
# \return [none]
#
# This is the 'main' function of the 'hint_loader' node. It sets up service clients for the '/armor_interface_srv' and '/oracle_solution' services. 
# Additionally, it initializes the '/hint_loader_service' service server.
#
def main():
	global armor_client_
	global oracle_client_	
	rospy.init_node('oracle')
	print("'hint_loader_service' is live...")
	hint_loader_server = rospy.Service('/hint_loader_service', HintLoader, clbk_oracle_service)
	armor_client_ = rospy.ServiceProxy('/armor_interface_srv', ArmorDirective)
	oracle_client_ = rospy.ServiceProxy('/oracle_solution', Oracle)
	rospy.spin()

if __name__ == '__main__':
    main()
