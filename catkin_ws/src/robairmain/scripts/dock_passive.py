#!/usr/bin/env python

###########
# Imports #
###########

import rospy
import cv2
import cv2.aruco as aruco
import numpy as np

from math import *

from std_msgs.msg import String
from std_msgs.msg import Byte
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose

from robairmain.msg import MotorsInfo

###################
# Robot Constants #
###################

wheel_diameter = 125 		#Wheels diameter in mm
wheels_distance = 400		#The distance between the tow wheels in mm
camera_center_distance = 0.185 	#The distance between the centre en the marker in m
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
DK_SEEN = 2			#Seen state
DK_NOTSEEN = 3		#Not seen state
DK_INPROGRESS
DK_DOCKED = 4		#Docked state

dock_distance = 1	#The distance where the robot is too close to send a dock request
MarkerWidth = 0.08      #Marker width in meters

#Both are used to get the marker position in the camera coordinate system
mtx = np.array([[635.35746725, 0, 330.33237895], [ 0, 636.86233192, 229.39423206], [0, 0, 1]])	#This is the camera callibration matrix 
disp = np.array([0.08063491, -0.29704246, 0.00137873, -0.00190106, 0.08985177])					#This is the camera dispertion array

aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)	#Declare the markers dictionnary
parameters =  aruco.DetectorParameters_create()			#Declare the aruco parameters

#############
# Variables #
#############

motors_info = MotorsInfo()	#RobAIR motors informations
marker_pos = Pose()			#Marker position in the camera coordinate system
camera_pos = Pose()			#Camera position in the marker coordinate system
robair_pose = Pose()		#RobAIR position in the marker coordinate system
motors_cmd = Twist()		#RobAIR motors command

DockState = 0			#Actual RobAIR state for docking

##################
# MOVES Funtions #
##################

def turn(angle):	#Turn over himself (angle in degrees)(- clockwise)(+ anti-clockwise)
	global motors_cmd, DockState		#Use the global motors_cmd

	motors_start = motors_info		#Get the initial motors informations
	motors_cmd.linear.x = 0			#Reset the linear command
	error = angle*robot_perimeter*encoder_resolution/360 - ((motors_info.countR - motors_start.countR) - (motors_info.countL - motors_start.countL))/2	#Get the error between the reference and the system

	while(error < (-wheel_incrementation/100) or (wheel_incrementation/100) < error) and (DockState != DK_NOTDOCKED):	#While the error is too big and the robot stay in the same dock state
		
		rate.sleep()	#Wait for the next sampling
		error = angle*robot_perimeter*encoder_resolution/360 - ((motors_info.countR - motors_start.countR) - (motors_info.countL - motors_start.countL))/2	#Get the error between the reference and the system

		if(wheel_incrementation/4 < error):			#If big error and positive
			motors_cmd.angular.z = FAST_ANGLE_SPEED		#Turn anti-clockwise
		elif(error < -wheel_incrementation/4):			#If big error and negative
			motors_cmd.angular.z = -FAST_ANGLE_SPEED	#Turn clockwise
		elif(0 < error and error < wheel_incrementation/4):	#If small error and positive
			motors_cmd.angular.z = SLOW_ANGLE_SPEED		#Turn anti-clockwise
		elif(-wheel_incrementation/4 < error and error < 0):	#If small error and negative
			motors_cmd.angular.z = -SLOW_ANGLE_SPEED	#Turn clockwise

		send_cmd_vel(motors_cmd)	#Send the command

	motors_cmd.angular.z = 0		#Reset the command

	if (DockState == DK_NOTDOCKED):	#If the operation have been cancel
		return False		#Return False
	else:
		return True		#Return True

def move(distance):	#Move stright (distance in meters)(- backward)(+ forward)
	global motors_cmd, DockState		#Use the global motors_cmd
	distance = distance * 1000	#Get the distance in mm
	motors_cmd.angular.z = 0	#Reset the angular command
	
	motors_start = motors_info		#Get the initial motors informations
	error = (distance*encoder_resolution) - ((motors_info.countR - motors_start.countR) + (motors_info.countL - motors_start.countL))/2	#Get the error between the reference and the system

	while((error < (-wheel_incrementation/50)) or ((wheel_incrementation/50) < error)) and (DockState != DK_NOTDOCKED and DockState != DK_DOCKED):	#While the error is too big and the robot stay in the same dock state
		
		rate.sleep()	#Wait for the next sampling
		error = (distance*encoder_resolution) - ((motors_info.countR - motors_start.countR) + (motors_info.countL - motors_start.countL))/2	#Get the error between the reference and the system
		
		if(wheel_incrementation < error):			#If big error and positive
			motors_cmd.linear.x = FAST_DISTANCE_SPEED	#Move forward
		elif(error < -wheel_incrementation):			#If big error and negative
			motors_cmd.linear.x = -FAST_DISTANCE_SPEED	#Move backward
		elif(0 < error and error < wheel_incrementation):	#If small error and positive
			motors_cmd.linear.x = SLOW_DISTANCE_SPEED	#Move forward
		elif(-wheel_incrementation < error and error < 0):	#If small error and negative
			motors_cmd.linear.x = -SLOW_DISTANCE_SPEED	#Move backward
		
		send_cmd_vel(motors_cmd)	#Send the command

	motors_cmd.linear.x = 0			#Reset the command

	if (DockState == DK_NOTDOCKED):	#If the operation have been cancel
		return False		#Return False
	else:
		return True		#Return True

##################
# CAMERA Funtion #
##################

def GetPose(cap):	#Get RobAIR position in screen coordinate
	global camera_pos, robair_pos	#Use the global camera_pos

	ret, image = cap.read()									#Save a picture from the video capture
   	gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)	#Transform into grey scale image
	corners, ids, rejectedImgPoints = aruco.detectMarkers(gray_image, aruco_dict, parameters=parameters)		#Detect markers on the picture and save IDs and cornes beloning to each ID
	
	if(isinstance(ids,np.ndarray)):		#If marker detected
		
		rvec, tvec = aruco.estimatePoseSingleMarkers(corners, MarkerWidth, mtx, disp)[0:2]	#Get the translation and rotation vector
		tvec = tvec[0][0]   			#Get the translation vector in meters
		rvec = rvec[0][0]   			#Get the rotation vector in radians
		rmat = cv2.Rodrigues(rvec)[0]	#Get the rotation matrix in radians	
		tvec = np.linalg.inv(tvec)		#Get the camera position in the marker coordinate system

		camera_pos.orientation.y = atan2(-rmat[2][0],sqrt(rmat[2][1]**2 + rmat[2][2]**2))	#Get the y marker orientation (in radians)
	
		camera_pos.position.x = tvec[0]	#Get the x marker position (in meters)
		camera_pos.position.z = tvec[2]	#Get the z marker position (in meters)

		robair_pos.position.x = camera_pos.position.x + sin(camera_pos.orientation.y)*camera_center_distance	#Get the x robair position (in meters)
		robair_pos.position.z = camera_pos.position.z + cos(camera_pos.orientation.y)*camera_center_distance	#Get the z robair position (in meters)

		return True		#Return true because a marker have been detected

	else:
		return False		#Return false because no marker detected

################
# DOCK Funtion #
################

def start_docking_camera():	#Start docking function
	
	cap = cv2.VideoCapture(0)			#Start a video frame

	State = DockState
	lastState = DockState

	while(DockState != DK_NOTDOCKED and DockState != DK_DOCKED):
		
		rate.sleep()	#Wait for the next sampling

		if(GetPose(cap) == True):	#If the marker is detected
			State = DK_SEEN

		else:
			State = DK_NOTSEEN

		if(LastState != State):
			send_dockstate(NOT_DOCKED)
			LastState = State

	cap.release()				#Stop the video frame

def start_docking_robair():

	while not rospy.is_shutdown():

		while(DockState != DK_WANTTODOCK):	#Wait for a dock request
			rate.sleep()					#Do nothing

		pub_log.publish('Start docking')	#Log info for ROS network

		while(DockState == DK_WANTTODOCK):	#Wait for an answer
			rate.sleep()			#Do nothing

		count = 0				#Reset counter
		motors_cmd.angular.z = SLOW_ANGLE_SPEED	#Turn anti-clockwise
		motors_cmd.linear.x = 0			#On himself
	
		while(DockState == DK_NOTSEEN and count < 200):	#While the robot is not seen and no timeout

			rate.sleep()			#Wait for the next sampling
			send_cmd_vel(motors_cmd)	#Send the command
			count += 1			#Incrementation

		if(DockState == DK_NOTSEEN):		#If the robot isn't seen yet
			send_dockstate(DK_NOTDOCKED)	#Cancel docking operation
			return False					#Return False

		while(DockState != DK_NOTDOCKED and DockState != DK_DOCKED and robair_pos.position.z < dock_distance):	#If the robot is too close

			rate.sleep()	#Wait for the next sampling

			X = 0
			Z = dock_distance * 2

			distance = sqrt((Z - robair_pos.position.z)**2+(X-robair_pos.position.x)**2)	#Get the distance between the objectif and the actual position
			angle2 = -atan2(robair_pos.position.x, dock_distance - robair_pos.position.z)*180/pi	#Get the angle for the objectif
			angle1 = -robair_pos.orientation.y*180/pi - angle2					#Get the angle for the actual

			turn(angle1)	#Turn
			move(distance)	#Move
			turn(angle2)	#Turn

		while(DockState != DK_NOTDOCKED and DockState != DK_DOCKED):	#While the robot is not docked

			rate.sleep()	#Wait for the next sampling

			if(DockState == DK_SEEN):

				sat_angle = sat(3 - camera_pos.position.z*2,0.17,1)
				target_angle = -sat(camera_pos.position.x * 5, -sat_angle, sat_angle)	#Get the target angle
				error_angle = target_angle - robair_pos.orientation.y

				motors_cmd.linear.x = -sat(SLOW_DISTANCE_SPEED/2 + camera_pos.position.z*SLOW_DISTANCE_SPEED/2,0,SLOW_DISTANCE_SPEED)	#Backward
				motors_cmd.angular.z = sat(error_angle*FAST_ANGLE_SPEED/0.52,-FAST_ANGLE_SPEED,FAST_ANGLE_SPEED)

			elif(DockState == DK_NOTSEEN):

				#motors_cmd.linear.x = 0
				#motors_cmd.angular.z = 0
				pass

			send_cmd_vel(motors_cmd)	#Send the command

			if(camera_pos.position.z < 0.24):
				
				send_dockstate(DK_DOCKED)	#The robot is docked

		motors_cmd.linear.x = 0		#Reset the linear command 
		motors_cmd.angular.z = 0	#Reset the angular command 

		pub_log.publish('Stop docking')	#Log info for ROS network

def sat(value, minimum, maximum):	#Saturation function
	if(value < minimum):		#If the value is under the minimum
		return minimum		#Set minimum
	elif(value > maximum):		#If the value is over the maximum
		return maximum		#Set maximum
	return value			#Stay the same

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
	global motors_info	#Use the global motors_info

	if(isinstance(data,MotorsInfo)):	#If data is type of MotorsInfo

		if(DockState != DK_NOTDOCKED and DockState != DK_DOCKED):

			if(abs(data.countR - motors_info.countR) > 10000):		#If the right counter change sign
				data.countR = motors_info.countR + 32768 - data.countR	#Change sign and add the difference

			if(abs(data.countL - motors_info.countL) > 10000):		#If the right counter change sign
				data.countL = motors_info.countL + 32768 - data.countL	#Change sign and add the difference

		motors_info = data		#Get motors info

def receive_dockstate(data):		#Receive dock state
	global DockState		#Use the global DockState and motors_cmd

	if (data.data == DK_WANTTODOCK):	#If the robot want to dock
		if(DockState == DK_NOTDOCKED):	#If the robot is not docked
			DockState = data.data		#Get the dock state
			start_docking_camera()		#Start the docking algorithme for the camera
	else:
		DockState = data.data		#Get the dock state

############################
# Subscribers & Publishers #
############################

rospy.Subscriber("motors_info",MotorsInfo,receive_motors_info)	#Subscribe to "motors_info" topic
rospy.Subscriber("dockstate",Byte,receive_dockstate)		#Subscribe to "dockstate" topic
pub_dock = rospy.Publisher('dockstate',Byte, queue_size=10)	#"dockstate" topic object
pub_vel = rospy.Publisher('cmd_vel',Twist, queue_size=10)	#"cmd_vel" topic object
pub_log = rospy.Publisher('log',String, queue_size=10)		#"log" topic object

########
# MAIN #
########

if __name__ == '__main__':	#If the file is executed for the first time
	
	try:
		rospy.init_node('dock_passive', anonymous=True)	#Initialise the node
		rate = rospy.Rate(10)			#Rate set to 10Hz
		pub_log.publish('Node "dock_passive" initialized')	#Log info for ROS network

		start_docking_robair()		#Start the docking algorithme for the robair
		rospy.spin()				#Wait for an event

  	except rospy.ROSInterruptException:

		rospy.logerr('error')	#Log info error

