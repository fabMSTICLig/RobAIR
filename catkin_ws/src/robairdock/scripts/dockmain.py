#!/usr/bin/env python

###########
# Imports #
###########

import rospy
import cv2
import cv2.aruco as aruco
import numpy as np

from math import *
from std_msgs.msg import Byte
from std_msgs.msg import String
from geometry_msgs.msg import Pose

###################
# Robot Constants #
###################
			
MarkerWidth = 0.08             	#Marker width in meters
robot_ID = 0

##################
# Dock Constants #
##################

DK_NOTDOCKED = 0	#Not docked state
DK_WANTTODOCK = 1	#Want to docked state
DK_NOTSEEN = 2		#Not seen state
DK_SEEN = 3		#Seen state
DK_DOCKED = 4		#Docked state


#For a RaspberryPi camera v2.1
mtx =  np.array([[505.62638698, 0, 326.44665333], [0, 506.57448647, 228.39570037],[0, 0, 1]])		#Camera matrix
disp = np.array([1.55319525e-01, 4.30522297e-02, -2.08579382e-04, -3.47100297e-03, -1.37788831e+00])	#Distortion matrix

aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)	#Declare the markers dictionnary
parameters =  aruco.DetectorParameters_create()		#Declare the aruco parameters

#############
# Variables #
#############

marker_pos = Pose()	#RobAIR position in the camera coordinate system

DockState = 0		#Actual RobAIR state for docking

#############
# Functions #
#############

def GetPose(cap):		#Get RobAIR position in screen coordinate
	global marker_pos	#Use the global marker_pos

	ret, image = cap.read()					#Save a picture from the video capture
   	gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)	#Transform into grey scale image
	corners, ids, rejectedImgPoints = aruco.detectMarkers(gray_image, aruco_dict, parameters=parameters)		#Detect markers on the picture and save IDs and cornes beloning to each ID
	
	if(isinstance(ids,np.ndarray)):		#If marker detected
		
		if(robot_ID in ids):			#If the base ID is detected

			rvec, tvec = aruco.estimatePoseSingleMarkers(corners, MarkerWidth, mtx, disp)[0:2]	#Get the translation and rotation vector

			index = np.where(ids == robot_ID)[0][0]	#Get the robot index

			tvec = tvec[index][0]   		#Get the translation vector in meters
			rvec = rvec[index][0]   		#Get the rotation vector

			rmat = cv2.Rodrigues(rvec)[0]		#Get the rotation matrix
				
			marker_pos.orientation.y = -atan2(-rmat[2][0],sqrt(rmat[2][1]**2 + rmat[2][2]**2))	#Get the y marker orientation (in radians)
			marker_pos.position.x = tvec[0]	#Get the x marker position (in meters)
			marker_pos.position.z = tvec[2]	#Get the z marker position (in meters)

			return True		#Return true because a marker have been detected

	else:	#If no marker detected
		return False		#Return false because no marker detected


def start_docking():		#Start docking function

	cap = cv2.VideoCapture(0)	#Start a video frame

	State = DockState		#State initialisation
	LastState = DockState		#Last statye initialization

	while(DockState != DK_NOTDOCKED and DockState != DK_DOCKED):	#If the docking operation isn't finish

		rate.sleep()	#Wait for the next sampling

		if(GetPose(cap) == True):	#If marker detected

			State = DK_SEEN				#The futur state will be DK_SEEN
			send_marker_pos(marker_pos)		#Publish the position

		else:	#If no marker detected
			State = DK_NOTSEEN			#The futur state will be DK_NOTSEEN

		if(LastState != State):		#If the last state is different from this one
			send_dockstate(State)	#Send the new state
			LastState = State	#Last state and actual state are now the same
				
	cap.release()				#Stop the video frame

def sat(value, minimum, maximum):	#Saturation function
	if(value < minimum):		#If under the minimum
		return minimum		#If under the minimum
	elif(value > maximum):		#If over the maximum
		return maximum		#If under the maximum
	return value			#Return the same value

#################
# Send Funtions #
#################

def send_dockstate(data):	#Send dock state
	global DockState	#Use the global DockState

	DockState = data	#Save the dock state	
	pub_dock.publish(data)	#Publish the state

def send_marker_pos(data):	#Send RobAIR position

	pub_pos.publish(data)	#Publish the position

####################
# Receive Funtions #
####################

def receive_dockstate(data):	#Receive dock state
	global DockState	#Use the global DockState

	if(data.data == DK_WANTTODOCK):		#If dock request
		if(DockState == DK_NOTDOCKED):	#If the robot want to dock and it is not
			DockState = data.data	#Save the dock state
			start_docking()		#Start docking

	else:	#If no dock request
		DockState = data.data		#Save the dock state

############################
# Subscribers & Publishers #
############################

rospy.Subscriber("dockstate",Byte,receive_dockstate)			#Subscribe to "dockstate" topic
pub_dock = rospy.Publisher('dockstate',Byte, queue_size=10)		#"dockstate" topic object
pub_pos = rospy.Publisher('position',Pose, queue_size=10)		#"position" topic object
pub_log = rospy.Publisher('log',String, queue_size=10)			#"log" topic object for debug

########
# MAIN #
########

if __name__ == '__main__':		#If the file is executed for the first time
	
	try:
		rospy.init_node('dockmain', anonymous=True)	#Initialize the node
		rate = rospy.Rate(10)				#Rate set to 10Hz

		pub_log.publish('Node "dockmain" initialized')	#Log info for ROS network
		rospy.loginfo('Node "dockmain" initialized')	#Log info for computer only

		rospy.spin()					#Wait for an event

  	except rospy.ROSInterruptException:

		rospy.logerr('error')	#Log info error

