import cv2
import cv2.aruco as aruco
import numpy as np
from time import time
from math import *

#############
# Variables #
#############

cap = cv2.VideoCapture(0)                                                   #Start the video frame
count = 0

while(True):

    ret, image = cap.read()                                                                   #Take a picture from the video frame    
    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)          #Transform the image in grey scale
    cv2.imshow('frame',gray_image)
    result =cv2.waitKey(1)
    
    if(result == ord('s')):
        name = 'img_' + str(count)+'.jpg'
        print(name + ' saved')
        cv2.imwrite(name, gray_image)
        count +=1
            
    elif(result == ord('q')):
       break
 
cap.release()
cv2.destroyAllWindows()
