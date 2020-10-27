# Imports
import rospy
import numpy as np
import cv2
import cv2.aruco as aruco

aruco_ids = [50, 100, 150, 200]

# Dicionario aruco
aruco_dict  = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
parameters  = aruco.DetectorParameters_create()
parameters.minDistanceToBorder = 0
parameters.adaptiveThreshWinSizeMax = 1000

# Calibracao camera
calib_path  = "/home/borg/catkin_ws/src/robot202/ros/exemplos202/scripts/"
camera_matrix   = np.loadtxt(calib_path+'cameraMatrix_raspi.txt', delimiter=',')
camera_distortion   = np.loadtxt(calib_path+'cameraDistortion_raspi.txt', delimiter=',')

class ArucoTracker:

    def detect_id(self, frame):
        distancenp = None

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters = parameters)

        if ids is not None:

            aruco.drawDetectedMarkers(frame, corners, ids)

            ret = aruco.estimatePoseSingleMarkers(corners, 25, camera_matrix, camera_distortion)

            rvec, tvec = ret[0][0,0,:], ret[1][0,0,:]

            distancenp = np.linalg.norm(tvec)

        cv2.imshow("Aruco_image", frame)

        return ids, distancenp




        