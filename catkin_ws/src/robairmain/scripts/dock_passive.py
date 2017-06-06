#!/usr/bin/env python

###########
# Imports #
###########

import rospy
from math import *

from std_msgs.msg import String
from std_msgs.msg import Byte
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose

from robairdock.msg import MotorsInfo

###################
# Robot Constants #
###################

wheel_diameter = 125 		#Wheels diameter in mm
wheels_distance = 400		#The distance between the tow wheels in mm
marker_center_distance = 0.185 	#The distance between the centre en the marker in m
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
DK_INPROGRESS = 2	#In progress state
DK_DOCKED = 3		#Docked state

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
marker_pos_camera = Pose()		#Marker position in the camera coordinate system
marker_pos_robair = Pose()		#RobAIR position in the camera coordinate system
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
	global marker_pos_camera	#Use the global marker_pos_camera

	ret, image = cap.read()									#Save a picture from the video capture
   	gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)	#Transform into grey scale image
	corners, ids, rejectedImgPoints = aruco.detectMarkers(gray_image, aruco_dict, parameters=parameters)		#Detect markers on the picture and save IDs and cornes beloning to each ID
	
	if(isinstance(ids,np.ndarray)):		#If marker detected
		
		rvec, tvec = aruco.estimatePoseSingleMarkers(corners, MarkerWidth, mtx, disp)[0:2]	#Get the translation and rotation vector
		tvec = tvec[0][0]   			#Get the translation vector in meters
		rvec = rvec[0][0]   			#Get the rotation vector in radians
		rmat = cv2.Rodrigues(rvec)[0]	#Get the rotation matrix in radians
				
		marker_pos_camera.orientation.y = -atan2(-rmat[2][0],sqrt(rmat[2][1]**2 + rmat[2][2]**2))	#Get the y marker orientation (in radians)
		marker_pos_camera.position.x = tvec[0]	#Get the x marker position (in meters)
		marker_pos_camera.position.z = tvec[2]	#Get the z marker position (in meters)

		marker_pos_robair.position.x = marker_pos_camera.position.x				#Get the x robair position
		marker_pos_robair.position.z = marker_pos_camera.position.z + camera_center_distance	#Get the z robair position
		marker_pos_robair.orientation.y = marker_pos_camera.orientation.y			#Get the y robair orientation

		return True		#Return true because a marker have been detected

	else:
		return False		#Return false because no marker detected

################
# DOCK Funtion #
################

def start_docking():	#Start docking function
	
	pub_log.publish('Start docking')	#Log info for ROS network
	rospy.loginfo('Start docking')		#Log info for computer only
	cap = cv2.VideoCapture(0)		#Start a video frame
	count = 0				#Reset counter

	while(DockState != DK_NOTDOCKED and DockState != DK_DOCKED):
		
		rate.sleep()	#Wait for the next sampling

		if(GetPose(cap) == True):	#If the marker is detected

			if(DockState == DK_WANTTODOCK):

				send_dockstate(DK_INPROGRESS)

				X = marker_pos_robair.x + cos(marker_pos_robair.orientation.y)*dock_distance	#Get the X coordinate objective
				Z = marker_pos_robair.z + sin(marker_pos_robair.orientation.y)*dock_distance	#Get the Z coordinate objective

				distance = -sqrt(X**2 + Z**2)					#Get the distance between the objectif and the actual position

				angle1 = -arctan2(Z,X)						#Get the angle for the actual position
				angle2 =-angle1 + marker_pos_robair.orientation.y*180/pi	#Get the angle for the objectif
						
				turn(angle1)	#Turn
				move(distance)	#Move backward
				turn(angle2)	#Turn

			else:

				if(marker_pos_camera.position.x > 0 and marker_pos_robair.orientation.y < 0):	#marker on the right facing in
					motors_cmd.angular.z = 0

				elif(marker_pos_camera.position.x > 0 and marker_pos_robair.orientation.y > 0):	#marker on the right facing out
					motors_cmd.angular.z = -FAST_ANGLE_SPEED

				elif(marker_pos_camera.position.x < 0 and marker_pos_robair.orientation.y > 0):	#marker on the left facing in
					motors_cmd.angular.z = 0

				elif(marker_pos_camera.position.x < 0 and marker_pos_robair.orientation.y < 0):	#marker on the left facing out
					motors_cmd.angular.z = FAST_ANGLE_SPEED				

				send_cmd_vel(motors_cmd)	#Send the command

		else:
			if(DockState == DK_WANTTODOCK):	#If the robot want to dock
				
				if(count < 200):

					motors_cmd.angular.z = SLOW_ANGLE_SPEED	#Turn anti-clockwise
					motors_cmd.linear.x = 0			#On himself
	
					send_cmd_vel(motors_cmd)	#Send the command
					count += 1			#Incrementation

				else:
					send_dockstate(NOT_DOCKED)
			
			if (DockState == DK_INPROGRESS and marker_pos_camera.position.z < MarkerWidth*3):	#If we are in DK_INPROGRESS mode and RobAIR is too close
				send_dockstate(DK_DOCKED)						#Set the state to DK_DOCKED

	cap.release()				#Stop the video frame
	pub_log.publish('Stop docking')		#Log info for ROS network
	rospy.loginfo('Stop docking')		#Log info for computer only

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

		if(DockState == DK_TOOCLOSE):

			if(abs(data.countR - motors_info.countR) > 10000):		#If the right counter change sign
				data.countR = motors_info.countR + 32768 - data.countR	#Change sign and add the difference

			if(abs(data.countL - motors_info.countL) > 10000):		#If the right counter change sign
				data.countL = motors_info.countL + 32768 - data.countL	#Change sign and add the difference

		motors_info = data		#Get motors info

def receive_dockstate(data):		#Receive dock state
	global DockState		#Use the global DockState and motors_cmd

	if (data.data == DK_WANTTODOCK):	#If the robot want to dock
		if(DockState == DK_NOTDOCKED):	#If the robot is not docked
			DockState = data.data	#Get the dock state
			start_docking()		#Start docking
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

		rospy.loginfo('Node "dock_passive" initialized')	#Log info for computer only
		pub_log.publish('Node "dock_passive" initialized')	#Log info for ROS network

		rospy.spin()					#Wait for an event

  	except rospy.ROSInterruptException:

		rospy.logerr('error')	#Log info error

