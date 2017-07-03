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

from diagnostic_msgs.msg import KeyValue

from robairdock.msg import MotorsInfo

###################
# Robot Constants #
###################

MarkerWidth = 0.08      #Marker width in meters

wheel_diameter = 125 		#Wheels diameter in mm
wheels_distance = 400		#The distance between the tow wheels in mm
center_camera_distance = 0.185 	#The distance between the centre en the marker in m
center_electrodes_distance = 0.25 	#The distance between the centre and the marker in m
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
DK_SEEN = 3			#Seen state
DK_DOCKED = 4		#Docked state

dock_distance = 2*center_electrodes_distance	#The distance where the robot is too close to dock

dock_ID = 1		#The dock ID

#Both are used to get the marker position in the camera coordinate system
mtx = np.array([[635.35746725, 0, 330.33237895], [ 0, 636.86233192, 229.39423206], [0, 0, 1]])	#This is the camera callibration matrix 
disp = np.array([0.08063491, -0.29704246, 0.00137873, -0.00190106, 0.08985177])			#This is the camera dispertion array

aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)	#Declare the markers dictionnary
parameters =  aruco.DetectorParameters_create()		#Declare the aruco parameters

#############
# Variables #
#############

motors_info = MotorsInfo()	#RobAIR motors informations
marker_pos = Pose()		#Marker position in the camera coordinate system
camera_pos = Pose()		#Camera position in the marker coordinate system
robair_pos = Pose()		#RobAIR position in the marker coordinate system
motors_cmd = Twist()		#RobAIR motors command

DockState = 0			#Actual RobAIR state for docking

Last_Batt_Level = 255
Last_ROBAIR_Start_Docking_Command = "NONE"

##################
# MOVES Funtions #
##################

def turn(angle):	#Turn over himself (angle in degrees)(- clockwise)(+ anti-clockwise)
	global motors_cmd, DockState	#Use the global motors_cmd

	motors_start = motors_info	#Get the initial motors informations
	motors_cmd.linear.x = 0		#Reset the linear command

	while(DockState != DK_NOTDOCKED):	#While the error is too big and the robot stay in the same dock state
		
		rate.sleep()	#Wait for the next sampling
		
		system_angle = ((motors_info.countR - motors_start.countR) - (motors_info.countL - motors_start.countL))*180/robot_perimeter/encoder_resolution
		error_angle = angle - system_angle	#Get the error between the reference and the system
		motors_cmd.angular.z = sat(error_angle*FAST_ANGLE_SPEED/90,-FAST_ANGLE_SPEED,FAST_ANGLE_SPEED)

		if(-5 < error_angle and error_angle < 5):
			return True

		else:
			send_cmd_vel(motors_cmd)	#Send the command

	return False		#Return False

def move(distance):	#Move stright (distance in meters)(- backward)(+ forward)
	global motors_cmd, DockState		#Use the global motors_cmd
	distance = distance * 1000	#Get the distance in mm
	
	motors_start = motors_info		#Get the initial motors informations
	motors_cmd.angular.z = 0	#Reset the angular command

	while(DockState != DK_NOTDOCKED):	#While the error is too big and the robot stay in the same dock state
		
		rate.sleep()	#Wait for the next sampling

		system_distance = ((motors_info.countR - motors_start.countR) + (motors_info.countL - motors_start.countL))/2/encoder_resolution
		error_distance = distance - system_distance 	#Get the error between the reference and the system
		motors_cmd.linear.x = sat(error_distance*FAST_DISTANCE_SPEED/1000,-FAST_DISTANCE_SPEED,FAST_DISTANCE_SPEED)	#Move backward

		if(-10 < error_distance and error_distance < 10):
			return True
		else:
			send_cmd_vel(motors_cmd)	#Send the command

	return False		#Return False

##################
# CAMERA Funtion #
##################

def GetPose(cap):	#Get RobAIR position in screen coordinate
	global camera_pos, robair_pos	#Use the global camera_pos

	ret, image = cap.read()					#Save a picture from the video capture
   	gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)	#Transform into grey scale image
	corners, ids, rejectedImgPoints = aruco.detectMarkers(gray_image, aruco_dict, parameters=parameters)		#Detect markers on the picture and save IDs and cornes beloning to each ID

	if(isinstance(ids,np.ndarray)):		#If marker detected
	
		if(dock_ID in ids):				#If the base ID is detected

			rvec, tvec = aruco.estimatePoseSingleMarkers(corners, MarkerWidth, mtx, disp)[0:2]	#Get the translation and rotation vector
			
			index = np.where(ids == dock_ID)[0][0]	#Get the robot index

			tvec = tvec[index][0]			#Get the translation vector in meters
			rvec = rvec[index][0]			#Get the rotation vector
			
			rmat = cv2.Rodrigues(rvec)[0]	#Get the rotation matrix

			module = sqrt(tvec[0]**2+tvec[1]**2 + tvec[2]**2)          #Get the module
			alpha = -atan2(-rmat[2][0],sqrt(rmat[2][1]**2+rmat[2][2]**2)) #Get the y orientation
			
			beta = atan2(tvec[0],tvec[2])
			teta = alpha+beta

			tvec[0] = sin(teta) * module
			tvec[2] = cos(teta) * module
			
			camera_pos.orientation.y = -alpha	#Get the y marker orientation (in radians)
			camera_pos.position.x = tvec[0]		#Get the x marker position (in meters)
			camera_pos.position.z = tvec[2]		#Get the z marker position (in meters)

			robair_pos.orientation.y = -alpha	#Get the y marker orientation (in radians)
			robair_pos.position.x = camera_pos.position.x + sin(camera_pos.orientation.y)*center_camera_distance	#Get the x robair position (in meters)
			robair_pos.position.z = camera_pos.position.z + cos(camera_pos.orientation.y)*center_camera_distance	#Get the z robair position (in meters)

			return True		#Return true because a marker have been detected

	else:
		return False		#Return false because no marker detected

################
# DOCK Funtion #
################

def start_docking_camera():	#Start docking function (camera part)
	
	cap = cv2.VideoCapture(0)	#Start a video frame

	State = DockState	#Set the dock state
	LastState = DockState	#Set the dockstate

	while(DockState != DK_NOTDOCKED and DockState != DK_DOCKED):	#While the robot is docking
		
		rate.sleep()			#Wait for the next sampling

		if(GetPose(cap) == True):	#If the marker is detected
			State = DK_SEEN			#The next state will be DK_SEEN

		else:
			State = DK_NOTSEEN	#The next state will be DK_NOTSEEN

		if(LastState != State):		#If the new state is differtent from the last state
			send_dockstate(State)	#Send dock state
			LastState = State		#Save the last state

	cap.release()				#Stop the video frame

def start_docking_robair():	#Start docking function (robair part)

	while not rospy.is_shutdown():			#While the node is node down

		while(DockState != DK_WANTTODOCK):	#Wait for a dock request
			rate.sleep()			#Do nothing

		pub_log.publish('Start docking')	#Log info for ROS network

		while(DockState == DK_WANTTODOCK):	#Wait for an answer
			rate.sleep()			#Do nothing

		count = 0				#Reset counter
		motors_cmd.angular.z = SLOW_ANGLE_SPEED	#Turn anti-clockwise
		motors_cmd.linear.x = 0			#On himself
	
		while(DockState == DK_NOTSEEN and count < 200):	#While the robot is not seen and no timeout

			rate.sleep()				#Wait for the next sampling
			send_cmd_vel(motors_cmd)	#Send the command
			count += 1					#Incrementation

		if(DockState == DK_NOTSEEN):		#If the robot isn't seen yet
			send_dockstate(DK_NOTDOCKED)	#Cancel docking operation
			return False			#Return False

		while(DockState != DK_NOTDOCKED and DockState != DK_DOCKED and robair_pos.position.z < dock_distance):	#If the robot is too close

			rate.sleep()	#Wait for the next sampling

			X = 0			#Set the x objective
			Z = dock_distance * 2	#Set the z objective

			distance = sqrt((Z - robair_pos.position.z)**2+(X-robair_pos.position.x)**2)		#Get the distance between the objectif and the actual position
			angle2 = -atan2(robair_pos.position.x, dock_distance - robair_pos.position.z)*180/pi	#Get the angle for the objectif
			angle1 = -robair_pos.orientation.y*180/pi - angle2					#Get the angle for the actual

			print(angle1)	#Turn
			print(distance)	#Move
			print(angle2)	#Turn

			turn(angle1)	#Turn
			move(distance)	#Move
			turn(angle2)	#Turn

		count = 0	#Set count to 0

		while(DockState != DK_NOTDOCKED and DockState != DK_DOCKED):	#While the robot is not docked

			rate.sleep()	#Wait for the next sampling

			if(DockState == DK_SEEN):	#If the robot is seen

				count = 0		#Reset count

				sat_angle = sat(3 - camera_pos.position.z*2,0.1745,0.5235)	#Get the sat angle
				gain = sat(7 - camera_pos.position.z*2,1,5)		#Get the gain
				target_angle = -sat(camera_pos.position.x * gain, -sat_angle, sat_angle)	#Get the target angle
				
				print(target_angle*180/pi)
				print(camera_pos.orientation.y*180/pi)
				print("")

				error_angle = target_angle - camera_pos.orientation.y	#Get the error

				motors_cmd.linear.x = sat(error_angle*SLOW_DISTANCE_SPEED/0.3490,0,SLOW_DISTANCE_SPEED)-SLOW_DISTANCE_SPEED		#Linear enslavement with angle									#Linear enslavement with position														#Get the slowest
				motors_cmd.angular.z = sat(error_angle*FAST_ANGLE_SPEED/0.52,-FAST_ANGLE_SPEED,FAST_ANGLE_SPEED)	#Angular enslavement

			elif(DockState == DK_NOTSEEN):	#If the robot is not seen
				count+=1		#Incrementation

				if(count >= 10):	#If the robot is not seen from 2 sec
					motors_cmd.linear.x = 0		#Stop
					motors_cmd.angular.z = 0	#Stop

				if(count >= 100):	#If the robot is not seen from 12 sec
					send_dockstate(DK_NOTDOCKED)	#Abort docking

			send_cmd_vel(motors_cmd)	#Send the command

			if(camera_pos.position.z < (center_electrodes_distance-center_camera_distance)*3):	#If the robot is close enough
				
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

		if(abs(data.countR - motors_info.countR) > 10000):		#If the right counter change sign
			data.countR = motors_info.countR + 32768 - data.countR	#Change sign and add the difference

		if(abs(data.countL - motors_info.countL) > 10000):		#If the right counter change sign
			data.countL = motors_info.countL + 32768 - data.countL	#Change sign and add the difference

	motors_info = data		#Get motors info

def receive_battery_level(data):	#Receive battery level information
	global Last_Batt_Level

	if(Last_Batt_Level != data.data):
		
		Last_Batt_Level = data.data

		test = KeyValue()
		test.key = "ROBAIR_Batt_Level"
		test.value = str(data.data)
		pub_iot.publish(test)

	if(DockState == DK_NOTDOCKED and data.data < 20):	#If we are not docked and low battery level	
		#send_dockstate(DK_WANTTODOCK)			#Send a dock request
		pass

def receive_dockstate(data):		#Receive dock state
	global DockState		#Use the global DockState and motors_cmd

	if (data.data == DK_WANTTODOCK):	#If the robot want to dock
		if(DockState == DK_NOTDOCKED):	#If the robot is not docked
			DockState = data.data	#Set the dock state
			start_docking_camera()	#Start the docking algorithme for the camera

	else:
		DockState = data.data		#Get the dock state

def receive_iot_updates(data):
	global Last_ROBAIR_Start_Docking_Command

	if(data.key == "ROBAIR_Start_Docking" and Last_ROBAIR_Start_Docking_Command != data.value):

		Last_ROBAIR_Start_Docking_Command = data.value

		if(data.value == "ON"):				#If receive ON
			send_dockstate(DK_WANTTODOCK)	#Send a dock request
			
		elif(data.value == "OFF"):			#If receive OFF
			send_dockstate(DK_NOTDOCKED)	#Cancel docking operation

############################
# Subscribers & Publishers #
############################

rospy.Subscriber("motors_info",MotorsInfo,receive_motors_info)	#Subscribe to "motors_info" topic
rospy.Subscriber("battery_level",Int32,receive_battery_level)	#Subscribe to "motors_info" topic
rospy.Subscriber("dockstate",Byte,receive_dockstate)		#Subscribe to "dockstate" topic
rospy.Subscriber("iot_updates",KeyValue,receive_iot_updates)		#Subscribe to "iot_updates" topic
pub_iot = rospy.Publisher('iot_command',KeyValue, queue_size=10)	#"dockstate" iot_command topic
pub_dock = rospy.Publisher('dockstate',Byte, queue_size=10)	#"dockstate" topic object
pub_vel = rospy.Publisher('cmd_vel',Twist, queue_size=10)	#"cmd_vel" topic object
pub_log = rospy.Publisher('log',String, queue_size=10)		#"log" topic object

########
# MAIN #
########

if __name__ == '__main__':	#If the file is executed for the first time
	
	try:
		rospy.init_node('dock_passive', anonymous=True)		#Initialise the node
		rate = rospy.Rate(10)					#Rate set to 10Hz
		pub_log.publish('Node "dock_passive" initialized')	#Log info for ROS network

		start_docking_robair()		#Start the docking algorithme for the robair
		rospy.spin()			#Wait for an event

  	except rospy.ROSInterruptException:

		rospy.logerr('error')	#Log info error

