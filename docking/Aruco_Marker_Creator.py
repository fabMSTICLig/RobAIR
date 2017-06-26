import cv2
import cv2.aruco as aruco
 
aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)	#Load the aruco dictionnary
file_path = "../Lib/Markers/"				#Set the file path
ID_min = 0		#Start ID
ID_max = 10		#End ID

for ID in range(ID_min, ID_max):	#Loop

    file_name = "6X6_250_ID" + str(ID) +".jpg"	#Set the full file name
    img = aruco.drawMarker(aruco_dict, ID, 500)	#Get the marker
    cv2.imwrite(file_path + file_name, img)	#Save the image

