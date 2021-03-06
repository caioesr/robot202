#! /usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
import numpy as np
import tf
import math
import cv2
import time
from geometry_msgs.msg import Twist, Vector3, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CompressedImage, LaserScan
from cv_bridge import CvBridge, CvBridgeError
import cormodule

bridge = CvBridge()

cv_image = None
velocity = 0.1
stop = False
media = []
centro = []
atraso = 1.5E9 # 1 segundo e meio. Em nanossegundos

area = 0.0 # Variavel com a area do maior contorno

# Só usar se os relógios ROS da Raspberry e do Linux desktop estiverem sincronizados. 
# Descarta imagens que chegam atrasadas demais
check_delay = False 

# A função a seguir é chamada sempre que chega um novo frame
def roda_todo_frame(imagem):
	global cv_image
	global media
	global centro

	now = rospy.get_rostime()
	imgtime = imagem.header.stamp
	lag = now-imgtime # calcula o lag
	delay = lag.nsecs
	if delay > atraso and check_delay==True:
		return 
	try:
		antes = time.clock()
		cv_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
		# cv_image = cv2.flip(cv_image, -1)
		media, centro, maior_area =  cormodule.identifica_cor(cv_image)
		depois = time.clock()
		cv2.imshow("Camera", cv_image)
	except CvBridgeError as e:
		print('ex', e)

def check_creeper(dado):

	global stop
	dado = list(np.array(dado.ranges))
	for i in dado[:15]:
		if i <= 0.5:
			stop = True
			return
	for i in dado[:345:-1]:
		if i <= 0.5:
			stop = True
			return
	stop = False

def scaneou(dado):
	# print("Faixa valida: ", dado.range_min , " - ", dado.range_max )
	# print("Leituras:")
	print(type(dado))
	
if __name__=="__main__":
	rospy.init_node("creeper")

	topico_imagem = "/camera/rgb/image_raw/compressed"

	recebedor = rospy.Subscriber(topico_imagem, CompressedImage, roda_todo_frame, queue_size=4, buff_size = 2**24)

	velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
	recebe_scan = rospy.Subscriber("/scan", LaserScan, check_creeper)

	try:
		while not rospy.is_shutdown():
			vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
			if len(media) != 0 and len(centro) != 0:
				if (media[0] > centro[0]):
					if stop:
						vel = Twist(Vector3(0,0,0), Vector3(0,0,-0.1))
					else:
						vel = Twist(Vector3(velocity,0,0), Vector3(0,0,-0.1))
				if (media[0] < centro[0]):
					if stop:
						vel = Twist(Vector3(0,0,0), Vector3(0,0,0.1))
					else:
						vel = Twist(Vector3(velocity,0,0), Vector3(0,0,0.1))
			velocidade_saida.publish(vel)
			rospy.sleep(0.1)

	except rospy.ROSInterruptException:
	    print("Ocorreu uma exceção com o rospy")
