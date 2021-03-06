#! /usr/bin/env python
# -*- coding:utf-8 -*-

from __future__ import print_function, division
import rospy
import numpy as np
import numpy
import tf
import math
import cv2
import time
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from numpy import linalg
from tf import transformations
from tf import TransformerROS
import tf2_ros
from geometry_msgs.msg import Twist, Vector3, Pose, Vector3Stamped

from nav_msgs.msg import Odometry
from std_msgs.msg import Header

import visao_module
from center_mass import center_mass
from tracker import tracker
from aruco import ArucoTracker
from claw import claw

bridge = CvBridge()

goal = ("orange", 11, "cat")

cv_image = None
media = []
centro = []
atraso = 1.5E9 # 1 segundo e meio. Em nanossegundos

low = np.array([22, 50, 50],dtype=np.uint8)
high = np.array([36, 255, 255],dtype=np.uint8)

v = 0.2
w = math.pi/12

tracker = tracker(v, w)
aruco_tracker = ArucoTracker(goal[1])
claw = claw()

cm_coords = None
cm_coords_creeper = None
center_image = None

area = 0.0 # Variavel com a area do maior contorno

# Só usar se os relógios ROS da Raspberry e do Linux desktop estiverem sincronizados. 
# Descarta imagens que chegam atrasadas demais
check_delay = False 

resultados = [] # Criacao de uma variavel global para guardar os resultados vistos

x = 0
y = 0
z = 0 
id = 0

frame = "camera_link"
# frame = "head_camera"  # DESCOMENTE para usar com webcam USB via roslaunch tag_tracking usbcam

tfl = 0

tf_buffer = tf2_ros.Buffer()


position = list()
inside_initial_area = True
first_movement = False
look_for_aruco = None

cm = center_mass(low, high)

if(goal[0] == "blue"):
    cm_creeper = center_mass(np.array([110, 50, 50], dtype=np.uint8), np.array([120, 255, 255],dtype=np.uint8))
elif(goal[0] == "orange"):
    cm_creeper = center_mass(np.array([0, 50, 50], dtype=np.uint8), np.array([10, 255, 255], dtype=np.uint8))
elif(goal[0] == "pink"):
    cm_creeper = center_mass(np.array([156,  50,  50], dtype=np.uint8), np.array([166, 255, 255], dtype=np.uint8))


def odometria(data):
    global position
    global inside_initial_area
    global first_movement
    global look_for_aruco
    global aruco_tracker

    position = [data.pose.pose.position.x, data.pose.pose.position.y]

    if aruco_tracker.done_turning == False:
        if(not first_movement):
            if((position[0] <= -0.5 or position[0] >= 0.5) or (position[1] <= -0.5 or position[1] >= 0.5)):
                first_movement = True
                inside_initial_area = False
        
        elif not inside_initial_area:
            if ((position[0] >= -0.25 and position[0] <= 0.25) and (position[1] >= -0.25 and position[1] <= 0.25)):
                look_for_aruco = True

    #print("Inside initial area: {0}\nFirst movement: {1}\nLooking for aruco: {2}".format(inside_initial_area, first_movement, look_for_aruco))
    #print("X = {0:.2f} Y = {1:.2f}".format(position[0], position[1]))

# A função a seguir é chamada sempre que chega um novo frame
def roda_todo_frame(imagem):
    global cv_image
    global media
    global centro
    global resultados
    global cm_coords
    global cm_coords_creeper
    global center_image
    global cm
    global cm_creeper

    now = rospy.get_rostime()
    imgtime = imagem.header.stamp
    lag = now-imgtime # calcula o lag
    delay = lag.nsecs
    # print("delay ", "{:.3f}".format(delay/1.0E9))
    if delay > atraso and check_delay==True:
        print("Descartando por causa do delay do frame:", delay)
        return 
    try:
        antes = time.clock()
        temp_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
        # Note que os resultados já são guardados automaticamente na variável
        # chamada resultados
        centro, saida_net, resultados =  visao_module.processa(temp_image)        
        for r in resultados:
            # print(r) - print feito para documentar e entender
            # o resultado            
            pass

        depois = time.clock()
        # Desnecessário - Hough e MobileNet já abrem janelas
        cv_image = saida_net.copy()
        center_image = tracker.get_center(cv_image)
        mask = cm.filter_color(cv_image)
        mask = cv2.blur(mask, (5, 5))
        cm_coords, mask_bgr = cm.center_of_mass_region(mask, 0, 160, cv_image.shape[1], cv_image.shape[0])
        tracker.crosshair(mask_bgr, center_image, 10, (0,255,0))

        mask_creeper = cm_creeper.filter_color(cv_image)
        mask_creeper = cv2.blur(mask_creeper, (5, 5))
        cm_coords_creeper, mask_bgr_creeper = cm_creeper.center_of_mass_region(mask_creeper, 0, 0, cv_image.shape[1], cv_image.shape[0])
        tracker.crosshair(mask_bgr_creeper, center_image, 10, (0,255,0))

        aruco_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
        aruco_tracker.detect_id(aruco_image, True)

        cv2.imshow("cv_image", mask_bgr)
        cv2.imshow("cv_image_creeper", mask_bgr_creeper)

        cv2.waitKey(1)
    except CvBridgeError as e:
        print('ex', e)
    
if __name__=="__main__":
    rospy.init_node("cor")

    topico_imagem = "/camera/image/compressed"

    recebedor = rospy.Subscriber(topico_imagem, CompressedImage, roda_todo_frame, queue_size=4, buff_size = 2**24)

    ref_odometria = rospy.Subscriber("/odom", Odometry, odometria)

    velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

    tfl = tf2_ros.TransformListener(tf_buffer) #conversao do sistema de coordenadas 
    tolerancia = 25

    # Exemplo de categoria de resultados
    # [('chair', 86.965459585189819, (90, 141), (177, 265))]

    try:
        
        while not rospy.is_shutdown():
            # for r in resultados:
            #     print(r)

            vel = None
            aruco_vel = aruco_tracker.get_velocity(math.pi / 8, 0.01)

            if not look_for_aruco:
                if aruco_vel == None:
                    if(center_image != None and cm_coords != None and cm_coords_creeper != None):
                        if(cm_coords_creeper[0] != 0 and cm_coords_creeper[1] != 0 and aruco_tracker.id_detected):
                            vel = tracker.get_velocity(center_image, cm_coords_creeper)
                        else:
                            vel = tracker.get_velocity(center_image, cm_coords)
                        velocidade_saida.publish(vel)
                else:
                    velocidade_saida.publish(aruco_vel)
            
            else:
                if aruco_vel == None:
                    vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, math.pi / 8))
                    velocidade_saida.publish(vel)
                else:
                    velocidade_saida.publish(aruco_vel)
                    look_for_aruco = not aruco_tracker.done_turning

            rospy.sleep(0.01)

            stop_vel = Twist(Vector3(0, 0, 0), Vector3(0, 0 ,0))
            velocidade_saida.publish(stop_vel)

            # claw.up_arm()
            # rospy.sleep(2)
            # claw.open_claw()
            # rospy.sleep(2)
            # claw.close_claw()
            # rospy.sleep(2)
            # claw.down_arm()

    except rospy.ROSInterruptException:
        print("Ocorreu uma exceção com o rospy")
