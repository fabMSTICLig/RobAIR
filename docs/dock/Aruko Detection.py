import cv2
import cv2.aruco as aruco
import numpy as np
from math import *
import matplotlib.pyplot as plt

#############
# Constants #
#############
 
markerLenght = 0.06             #6 cm width

mtx = np.array([[635.35746725, 0, 330.33237895], [ 0, 636.86233192, 229.39423206], [0, 0, 1]])
disp = np.array([0.08063491, -0.29704246, 0.00137873, -0.00190106, 0.08985177])

#############
# Variables #
#############

cap = cv2.VideoCapture(0)                                                   #Start the video frame

aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)     #Load the aruco dictionnary
parameters =  aruco.DetectorParameters_create()             #Load aruco parameters
 
while(True):
    
    ret, image = cap.read()                                                                   #Take a picture from the video frame    
    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)          #Transform the image in grey scale

    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray_image, aruco_dict, parameters=parameters) #List ids and the corners beloning to each marker

    if(isinstance(ids,np.ndarray)):		#If marker detected
 
        image = aruco.drawDetectedMarkers(image, corners)
        rvec, tvec = aruco.estimatePoseSingleMarkers(corners, markerLenght, mtx, disp)[0:2]
        rvec = rvec[0][0]   #Rotation matrix in radians
        tvec = tvec[0][0]   #Translation matrix in meters

        module = sqrt(tvec[0]**2+tvec[1]**2 + tvec[2]**2)*100
        rmat = cv2.Rodrigues(rvec)[0]
        teta = atan2(-rmat[2][0],sqrt(rmat[2][1]**2+rmat[2][2]**2))*180/pi
        
    cv2.imshow('frame',image)
    result =cv2.waitKey(1)
    
    if(result == ord('s')):

        if(isinstance(ids,np.ndarray)):		#If marker detected
            print("x = " + str(tvec[0]*100) + " cm\ny = " + str(tvec[1]*100) + "cm\nz = " + str(tvec[2]*100) + " cm\nTotal = " + str(module) + " cm\nAngle = " +str(teta))
            cv2.imwrite('test.jpg', image)

        elif ids == None:
            print("No marker detected")
            
    elif(result == ord('q')):
       break
 
cap.release()
cv2.destroyAllWindows()
