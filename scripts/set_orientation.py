#! /usr/bin/env python

## @package erl2
# \file set_orientation.py
# \brief This file contains the implementation for the set_orientation node.
# \author Muhammad Ali Haider Dar
# \version 1.0
# \date 12/08/2024
#
# \details
#
# Service : <BR>
# ° /armor_interface_srv
# ° /oracle_service
#  
# This node receives the desired orientation goal coordinates via a '/request_set_orientation' service request from the 'my_action' node.
# It calculates the necessary angular velocity for the robot to reach the target orientation and publishes the resulting command to the 'cmd_vel' topic.
#

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point, Pose
from erl2.srv import SetOrien
from nav_msgs.msg import Odometry
from tf import transformations
import math
import actionlib
import actionlib.msg
import motion_plan.msg

## A global integer variable used to store the robot's yaw value.
yaw_ = 0

## A global float variable used as a parameter for the robot's yaw precision.
yaw_precision_ = math.pi / 9  # +/- 20 degree allowed

## A global float variable used as a secondary parameter for the robot's yaw precision.
yaw_precision_2_ = math.pi / 90  # +/- 2 degree allowed

## A global variable for initializing the publisher for the '/cmd_vel' topic.
pub = None

## A global float variable used as a parameter for calculating the robot's new angular velocities.
kp_a = -3.0  

## A global float variable used as a parameter for calculating the robot's new linear velocities.
kp_d = 0.2

## A global float variable used to set the upper bound for the robot's angular velocity.
ub_a = 0.6

## A global float variable used to set the lower bound for the robot's angular velocity.
lb_a = -0.5

## A global float variable used to set the upper bound for the robot's linear velocity.
ub_d = 0.6

##
# \brief This is the callback function for the 'sub_odom' subscriber of the ROS topic '/odom'.
# \param msg Argument with data structure type Odometry  
# \return [none]
#
# This function processes the robot's odometry data received from the '/odom' topic,
# extracts the position and yaw, and stores them in the global variables 'position_' 
# and 'yaw_'.
#
def clbk_odom(msg):
    global position_
    global pose_
    global yaw_

    # position
    position_ = msg.pose.pose.position
    pose_ = msg.pose.pose

    # yaw
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2]
    #print(yaw_)
    
##
# \brief This function normalizes the robot's orientation angle relative to the goal orientation.
# \param angle Argument with data type 'float'
# \return Normalized angle value
#
# This function adjusts the robot's orientation angle to be within a standard range relative to the goal orientation.
#
def normalize_angle(angle):
    if(math.fabs(angle) > math.pi):
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle

##
# \brief This function adjusts the robot's yaw.
# \param des_pos Argument with data type 'float'
# \return [none]
#
# This function takes the desired position as input. It calculates the necessary angular velocity based on the 'yaw error' and then publishes it to the 'cmd_vel' topic.
#
def fix_yaw(goal):
		
		global yaw_, pub, yaw_precision_2_
		desired_yaw = goal
		print(math.atan2(2.0 - 0.0, 0.0 - 0.0))
		err_yaw = normalize_angle(desired_yaw - yaw_)
		
		while not (math.fabs(err_yaw) <= yaw_precision_2_):
				err_yaw = normalize_angle(desired_yaw - yaw_)
				rospy.loginfo(err_yaw)
				twist_msg = Twist()
				
				if math.fabs(err_yaw) > yaw_precision_2_:
						twist_msg.angular.z = kp_a*err_yaw
						if twist_msg.angular.z > ub_a:
								twist_msg.angular.z = ub_a
						elif twist_msg.angular.z < lb_a:
								twist_msg.angular.z = lb_a
				pub.publish(twist_msg)

			# state change conditions
			#if math.fabs(err_yaw) <= yaw_precision_2_:
		print ('Yaw error: [%s]' % err_yaw)
		return done()
		
##
# \brief This function sets the robot's angular velocities to zero.
# \param [none]   
# \return [none]
#
# This function stops the robot by setting its angular velocities to zero. It is typically used when the robot has successfully reached the desired orientation.
#
def done():
    twist_msg = Twist()
    twist_msg.angular.z = 0
    pub.publish(twist_msg)
    return True

##
# \brief This is the callback function for the 'request_set_orientation' service.
# 
# \return Bool
#
# This function responds to client requests by invoking the 'set_orientation' function to adjust the robot's orientation.
#
def set_orien_clbk(msg):
		success = fix_yaw(msg.goal)
		return success

##
# \brief The main function of the 'hint_collector' node.
# 
# \return Always returns 0, as this function cannot fail.
#
# This function initializes the ROS node, sets up the server for the '/request_set_orien' service, subscribes to the '/odom' topic, and initializes the publisher for the '/cmd_vel' topic.
#
def main():
	global pub
	rospy.init_node('set_orientation_server')
	pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
	sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
	set_orien_server = rospy.Service('/request_set_orientation',SetOrien, set_orien_clbk)
	print ('set orientation service is live...')
	rate = rospy.Rate(20)
	while not rospy.is_shutdown():
		rate.sleep()

if __name__ == '__main__':
    main()
