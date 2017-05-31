import cv2
import cv2.aruco as aruco
 
aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
file_path = "../Lib/Aruco Markers/"
ID_min = 0
ID_max = 250

for ID in range(ID_min, ID_max):

    file_name = "6X6_250_ID" + str(ID) +".jpg"
    img = aruco.drawMarker(aruco_dict, ID, 500)
    cv2.imwrite(file_path + file_name, img)

