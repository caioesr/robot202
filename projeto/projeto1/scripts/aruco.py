# Imports
import rospy
import math
from geometry_msgs.msg import Twist, Vector3
import numpy as np
import cv2
import cv2.aruco as aruco

min_dist = 215

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
    def __init__(self):
        self.total_ang = 0
        self.state = "Idle"

    def detect_id(self, frame, treat_ids):
        distancesnp = None

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.blur(gray, (5, 5))

        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters = parameters)

        if ids is not None:

            aruco.drawDetectedMarkers(frame, corners, ids)

            ret = aruco.estimatePoseSingleMarkers(corners, 25, camera_matrix, camera_distortion)

            rvec, tvec = ret[0][0,0,:], ret[1][0,0,:]

            distancesnp = np.linalg.norm(tvec)

            if treat_ids:

                if (ids[0] == 50 or ids[0] == 100 or ids[0] == 150) and distancesnp <= min_dist:
                    self.state = "Turn Back"
                elif ids[0] == 200 and distancesnp <= min_dist:
                    self.state = "Turn Right"

        cv2.imshow("Aruco_image", frame)

    def get_velocity(self, w, dt):
        if self.state == "Turn Back":
            return self.turn_back(w, dt)
        elif self.state == "Turn Right":
            return self.turn_right(w, dt)
        elif self.state == "Idle":
            return None

    def turn_back(self, w, dt):        
        if self.total_ang < math.pi:
            self.total_ang += w * dt
            return Twist(Vector3(0, 0, 0), Vector3(0, 0, w))
        else:
            self.total_ang = 0
            self.state = "Idle"    
    def turn_right(self, w, dt):
        if self.total_ang < math.pi / 4:
            self.total_ang += w * dt
            return Twist(Vector3(0, 0, 0), Vector3(0, 0, -w))
        else:
            self.total_ang = 0
            self.state = "Idle"