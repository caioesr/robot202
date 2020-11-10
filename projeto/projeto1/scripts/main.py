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
from creeper import creeper
from aruco import ArucoTracker
from claw import claw
inputlist = ["42069","blue","cat"]
bridge = CvBridge()

cv_image = None
crm= None
media = []
centro = []
atraso = 1.5E9 # 1 segundo e meio. Em nanossegundos

low = np.array([22, 50, 50],dtype=np.uint8)
high = np.array([36, 255, 255],dtype=np.uint8)

v = 1
w = math.pi/16

tracker = tracker(v, w)
aruco_tracker = ArucoTracker()
claw = claw()

creeper_coords = None
cm_coords = None
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

# A função a seguir é chamada sempre que chega um novo frame
def roda_todo_frame(imagem):
    global cv_image
    global media
    global centro
    global resultados
    global cm_coords
    global center_image
    global creeper_coords

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
        cm = center_mass(low, high)
        crm = creeper(inputlist)
        #cria a mascara e area da mascara para o centro de massa
        mask_cm = cm.filter_color(cv_image)
        cm_coords,mask_bgr_cm = cm.center_of_mass_region(mask_cm, 100, 175, cv_image.shape[1] - 100, cv_image.shape[0])
        tracker.crosshair(mask_bgr_cm, center_image, 10, (0,255,0))
        #cria a mascara e area da mascara para o creeper
        mask_crm= crm.filtra_creeper(cv_image)
        creeper_coords,mask_bgr_crm = crm.center_of_creeper_region(mask_crm, 0, 150, cv_image.shape[1] - 100, cv_image.shape[0])

        cv2.imshow("Mascara_crm",mask_bgr_crm)
        cv2.imshow("Mascara_cm",mask_bgr_cm)

        aruco_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
        aruco_tracker.detect_id(aruco_image, True)

        cv2.imshow("cv_image", cv_image)
        cv2.waitKey(1)
    except CvBridgeError as e:
        print('ex', e)
    
if __name__=="__main__":
    rospy.init_node("cor")

    topico_imagem = "/camera/image/compressed"

    recebedor = rospy.Subscriber(topico_imagem, CompressedImage, roda_todo_frame, queue_size=4, buff_size = 2**24)

    velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

    tfl = tf2_ros.TransformListener(tf_buffer) #conversao do sistema de coordenadas 
    tolerancia = 25 

    # Exemplo de categoria de resultados
    # [('chair', 86.965459585189819, (90, 141), (177, 265))]

    try:
        
        while not rospy.is_shutdown():
            # for r in resultados:
            #     print(r)

            aruco_vel = aruco_tracker.get_velocity(math.pi / 8, 0.01)

            if aruco_vel == None:
                if(center_image != None and creeper_coords[0] != 0 and creeper_coords[1] !=0):
                    vel = tracker.get_velocity(center_image,creeper_coords)
                    velocidade_saida.publish(vel)

                elif(center_image != None and cm_coords != None):
                    vel = tracker.get_velocity(center_image, cm_coords)
                    velocidade_saida.publish(vel)
            else:
                velocidade_saida.publish(aruco_vel)

            rospy.sleep(0.01)

            stop_vel = Twist(Vector3(0, 0, 0), Vector3(0, 0 ,0))
            velocidade_saida.publish(stop_vel)

    except rospy.ROSInterruptException:
        print("Ocorreu uma exceção com o rospy")
