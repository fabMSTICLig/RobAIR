#!/usr/bin/env python

###########
# Imports #
###########

import rospy

from math import *
from std_msgs.msg import String
from std_msgs.msg import Byte
from std_msgs.msg import Int32
from robairdock.msg import MotorsInfo
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose

###################
# Robot Constants #
###################

wheel_diameter = 125 		#Wheels diameter in mm
wheels_distance = 400		#The distance between the tow wheels in mm
marker_center_distance = 0.185 	#The distance between the centre en the marker in mm
wheel_incrementation = 976 	#Number of incrementaion in one wheel

wheel_perimeter = pi*wheel_diameter 				#Wheel perimeter in mm
robot_perimeter = pi*wheels_distance				#Robot perimeter in mm
encoder_resolution = wheel_incrementation/wheel_perimeter	#Incrementation per mm

SLOW_DISTANCE_SPEED = 0.05	#Linear speed when close to the objective
FAST_DISTANCE_SPEED = 0.2	#Linear speed when far from the objective

SLOW_ANGLE_SPEED = 0.05		#Angular speed when close to the objective
FAST_ANGLE_SPEED = 0.2		#Angular speed when far from the objective

##################
# Dock Constants #
##################

DK_NOTDOCKED = 0	#Not docked state
DK_WANTTODOCK = 1	#Want to dock state
DK_NOTSEEN = 2		#Not seen state
DK_TOOCLOSE = 3		#Too close state
DK_INPROGRESS = 4	#In progress state
DK_DOCKED = 5		#Docked state

dock_distance = 1	#The distance where the robot is too close to send a dock request

#############
# Variables #
#############

motors_info = MotorsInfo()	#RobAIR motors informations
marker_pos = Pose()		#RobAIR position in the camera coordinate system
motors_cmd = Twist()		#RobAIR motors command

DockState = 0		#Actual RobAIR state for docking
See = False

##################
# MOVES Funtions #
##################

def turn(angle):#, state = DockState): #Turn over himself (angle in degrees)(- clockwise)(+ anti-clockwise)
	global motors_cmd, DockState		#Use the global motors_cmd

	motors_start = motors_info		#Get the initial motors informations
	error = angle*robot_perimeter*encoder_resolution/360 - ((motors_info.countR - motors_start.countR) - (motors_info.countL - motors_start.countL))/2	#Get the error between the reference and the system

	while(error < (-wheel_incrementation/100) or (wheel_incrementation/100) < error) and (DockState != DK_NOTDOCKED):	#While the error is too big and the robot stay in the same dock state
		
		rate.sleep()	#Wait for the next sampling

		error = angle*robot_perimeter*encoder_resolution/360 - ((motors_info.countR - motors_start.countR) - (motors_info.countL - motors_start.countL))/2	#Get the error between the reference and the system

		if(wheel_incrementation/4 < error):			#If big error and positive
			motors_cmd.angular.z = FAST_ANGLE_SPEED		#Turn anti-clockwise
		elif(error < -wheel_incrementation/4):			#If big error and negative
			motors_cmd.angular.z = -FAST_ANGLE_SPEED		#Turn clockwise
		elif(0 < error and error < wheel_incrementation/4):	#If small error and positive
			motors_cmd.angular.z = SLOW_ANGLE_SPEED		#Turn anti-clockwise
		elif(-wheel_incrementation/4 < error and error < 0):	#If small error and negative
			motors_cmd.angular.z = -SLOW_ANGLE_SPEED		#Turn clockwise

		send_cmd_vel(motors_cmd)	#Send the command

	motors_cmd.angular.z = 0		#Reset the command

	if (DockState == DK_NOTDOCKED):
		return False
	else:
		return True

def move(distance):  #Move stright (distance in meters)(- backward)(+ forward)
	global motors_cmd, DockState		#Use the global motors_cmd
	distance = distance * 1000	#Get the distance in mm
	
	motors_start = motors_info		#Get the initial motors informations
	error = (distance*encoder_resolution) - ((motors_info.countR - motors_start.countR) + (motors_info.countL - motors_start.countL))/2	#Get the error between the reference and the system

	while((error < (-wheel_incrementation/50)) or ((wheel_incrementation/50) < error)) and (DockState != DK_NOTDOCKED):	#While the error is too big and the robot stay in the same dock state
		
		rate.sleep()	#Wait for the next sampling

		error = (distance*encoder_resolution) - ((motors_info.countR - motors_start.countR) + (motors_info.countL - motors_start.countL))/2	#Get the error between the reference and the system
		
		if(wheel_incrementation < error):			#If big error and positive
			motors_cmd.linear.x = FAST_DISTANCE_SPEED		#Move forward
		elif(error < -wheel_incrementation):			#If big error and negative
			motors_cmd.linear.x = -FAST_DISTANCE_SPEED		#Move backward
		elif(0 < error and error < wheel_incrementation):	#If small error and positive
			motors_cmd.linear.x = SLOW_DISTANCE_SPEED		#Move forward
		elif(-wheel_incrementation < error and error < 0):	#If small error and negative
			motors_cmd.linear.x = -SLOW_DISTANCE_SPEED		#Move backward
		
		send_cmd_vel(motors_cmd)	#Send the command

	motors_cmd.linear.x = 0		#Reset the command

	if(DockState == DK_NOTDOCKED):
		return False
	else:
		return True

#################
# Send Funtions #
#################

def send_dockstate(data):	#Send dock state
	global DockState	#Use the global DockState
	DockState = data	#Save the dock state

	pub_dock.publish(data)	#Publish the state

def send_cmd_vel(data):		#Send motors commands
	pub_vel.publish(data)	#Publish motors commands

####################
# Receive Function #
####################

def receive_motors_info(data):	#Receive motors informations
	global motors_info		#Use the global motors_info

	if(isinstance(data,MotorsInfo)):	#If data is type of MotorsInfo
		motors_info = data		#Get motors info


def receive_position(data):	#Receive RobAIR position in 
	global marker_pos, See		#Use the global marker_pos
	marker_pos = data	#Get the position

	if (DockState == DK_INPROGRESS):	#If it is the DK_INPROGRESS state 

		#Get the cmd_vel in function of marker_pos		

		if(marker_pos.position.x > 0):
			motors_cmd.angular.z = -FAST_ANGLE_SPEED
		else:
			motors_cmd.angular.z = FAST_ANGLE_SPEED

		motors_cmd.linear.x = 0.9*SLOW_DISTANCE_SPEED

		#Enslavement ?

		send_cmd_vel(motors_cmd)	#Send the command

	elif(DockState == DK_NOTSEEN):
		See = True

def receive_battery_level(data):	#Receive battery level information

	if(DockState == DK_NOTDOCKED and data.data < 15):	#If we are not docked and low battery level	
		send_dockstate(DK_WANTTODOCK)			#Send a dock request

def receive_dockstate(data):		#Receive dock state
	global DockState, motors_cmd, See	#Use the global DockState and motors_cmd
	DockState = data.data		#Get the dock state

	if (DockState == DK_NOTSEEN):	#If RobAIR is not seen

		count = 0				#Reset counter
		motors_cmd.angular.z = SLOW_ANGLE_SPEED	#Turn anti-clockwise
		See = False				#Reset the flag

		while(See == False):	#While the robot is not see

			rate.sleep()		#Wait for the next sampling
			send_cmd_vel(motors_cmd)	#Send the command
			count += 1		#Incrementation
		
		motors_cmd.angular.z = 0		#Stop turning

		if(count >= 200):		#If the robot has not been seen

			send_dockstate(DK_NOTDOCKED)	#Stop Docking

	elif (DockState == DK_TOOCLOSE):	#If the robot is too close
		
		robair_pos = marker_pos
		robair_pos.position.x = marker_pos.position.x + sin(marker_pos.orientation.y)*marker_center_distance
		robair_pos.position.z = marker_pos.position.z + sin(marker_pos.orientation.y)*marker_center_distance

		distance = -sqrt((dock_distance - robair_pos.position.z)**2+robair_pos.position.x**2)	#Get the distance between the objectif and the actual position
		angle2 = -atan2(robair_pos.position.x, dock_distance - robair_pos.position.z)*180/pi	#Get the angle for the objectif
		angle1 = robair_pos.orientation.y*180/pi - angle2					#Get the angle for the actual position
		
		pub_log.publish('Turn ' + str(angle1) + ' degrees')	#Log info for ROS network
		pub_log.publish('Move ' + str(distance) + ' meters')	#Log info for ROS network
		pub_log.publish('Turn ' + str(angle2) + ' degrees')	#Log info for ROS network

		turn(angle1)	#Turn
		move(distance)	#Move
		turn(angle2)	#Turn
    		
############################
# Subscribers & Publishers #
############################

rospy.Subscriber("motors_info",MotorsInfo,receive_motors_info)	#Subscribe to "motors_info" topic
rospy.Subscriber("battery_level",Int32,receive_motors_info)	#Subscribe to "motors_info" topic
rospy.Subscriber("position",Pose,receive_position)		#Subscribe to "position" topic
rospy.Subscriber("dockstate",Byte,receive_dockstate)		#Subscribe to "dockstate" topic
pub_dock = rospy.Publisher('dockstate',Byte, queue_size=10)	#"dockstate" topic object
pub_vel = rospy.Publisher('cmd_vel',Twist, queue_size=10)	#"cmd_vel" topic object
pub_log = rospy.Publisher('log',String, queue_size=10)		#"log" topic object

########
# MAIN #
########

if __name__ == '__main__':	#If the file is executed for the first time
	
	try:
		rospy.init_node('dockcommand', anonymous=True)	#Initialise the node
		rate = rospy.Rate(10)				#Rate set to 10Hz

		rospy.loginfo('Node "dockcommand" initialized')		#Log info for computer only
		pub_log.publish('Node "dockcommand" initialized')	#Log info for ROS network

		rospy.spin()						#Wait for an event

  	except rospy.ROSInterruptException:

		rospy.logerr('error')	#Log info error

