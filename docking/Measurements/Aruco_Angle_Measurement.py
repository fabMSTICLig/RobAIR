import cv2
import cv2.aruco as aruco
import numpy as np
from math import *
import matplotlib.pyplot as plt

#################
#   Constantes  #
#################
 
markerLenght = 0.06             #Marker width

#For a RaspberryPi camera v1.3
mtx_v1 = np.array([[635.35746725, 0, 330.33237895], [ 0, 636.86233192, 229.39423206], [0, 0, 1]])	#Camera matrix
disp_v1 = np.array([0.08063491, -0.29704246, 0.00137873, -0.00190106, 0.08985177])			#Distortion matrix

#For a RaspberryPi camera v2.1
mtx_v2 =  np.array([[505.62638698, 0, 326.44665333], [0, 506.57448647, 228.39570037],[0, 0, 1]])	#Camera matrix
disp_v2 = np.array([1.55319525e-01, 4.30522297e-02, -2.08579382e-04, -3.47100297e-03, -1.37788831e+00])	#Distortion matrix

start_angle = -90	#Start angle
end_angle = 90		#End angle
step_angle = 5		#Step angle

angle_tab = []		#X tab
angle_error_tab = []	#Y tab

#################
#   Variables   #
#################

angle = start_angle					#Set the actual angle

cap = cv2.VideoCapture(0)                             	#Start the video frame
aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)  	#Load the aruco dictionnary
parameters =  aruco.DetectorParameters_create()        	#Load aruco parameters
 
while(True):	#Infinite loop
    
    ret, image = cap.read()					#Take a picture from the video frame    
    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)        #Transform the image in grey scale

    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray_image, aruco_dict, parameters=parameters) #List ids and the corners beloning to each marker

    if(isinstance(ids,np.ndarray)):		#If marker detected:
 
        image = aruco.drawDetectedMarkers(image, corners)	#Draw marker outline
        rvec, tvec = aruco.estimatePoseSingleMarkers(corners, markerLenght, mtx_v2, disp_v2)[0:2]	#Get matrix
        rvec = rvec[0][0]   #Get rotation vectors
        tvec = tvec[0][0]   #Get translation vectors in meters
        rmat = cv2.Rodrigues(rvec)[0]	#Get rotation matrix

        teta = atan2(-rmat[2][0],sqrt(rmat[2][1]**2+rmat[2][2]**2))*180/pi	#Get the y orientation
        
    cv2.imshow('frame',image)	#Show image
    result =cv2.waitKey(1)    	#Get pressed key

    if(result == ord('s')):	#If s pressed
  
        if(isinstance(ids,np.ndarray)):		#If marker detected:
            print("Measured angle = " + str(teta) + "° for "+ str(angle) + "°.")	#Log info
            angle_error_tab.append(abs(angle-teta))					#Save error
            angle_tab.append(angle)							#Save angle

        else:	#If no marker detected
            print("No marker detected for " + str(angle) + "°.")	#Log info

        angle += step_angle	#Incrementation
            
        if(angle > end_angle):	#If finish
            break		#Break
    
    elif(result == ord('q')):	#If q pressed
       break			#Break
 
cap.release()			#Stop video frame
cv2.destroyAllWindows()		#Close image

plt.plot(angle_tab,angle_error_tab,'ro')			#Create a graph
plt.xlabel('Angle reel (en degres)')				#Set x label
plt.ylabel('Erreur absolue sur l\'angle mesure (en degres)')	#Set y label
plt.grid()	#Set grid
plt.show()	#Show graph
