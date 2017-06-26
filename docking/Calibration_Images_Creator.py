import cv2
import cv2.aruco as aruco
import numpy as np
from time import time
from math import *

#############
# Variables #
#############

path = 'Calibration_Images/' + input("Enter your camera name :\n(Exemple : RaspberryPi_v1.3)") + '/'	#Set the path
cap = cv2.VideoCapture(0)	#Start the video frame
count = 0			#Set count to 0

while(True):	#Infinite loop

    ret, image = cap.read()					#Take a picture from the video frame    
    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)	#Transform the image in grey scale
    cv2.imshow('frame',gray_image)	#Show the image
    result =cv2.waitKey(1)		#Get pressed key
    
    if(result == ord('s')):	#If s pressed
        name = 'img_' + str(count)+'.jpg'	#Set the full name
        print(name + ' saved')			#Log info
        cv2.imwrite(path + name, gray_image)		#Save the image
        count +=1				#Incrementation
            
    elif(result == ord('q')):	#If q pressed
       break			#Break
 
cap.release()			#Stop video frame
cv2.destroyAllWindows()		#Close image
