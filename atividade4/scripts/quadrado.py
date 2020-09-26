#! /usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
from geometry_msgs.msg import Twist, Vector3
import math

print("Inicializando a rotina!")

v = 0.5                     # Velocidade linear
w = math.pi / 2             # Velocidade angular

movements = [v, w, v, w, v, w, v, w] # Lista de movimentos

i = 0   # Iterador

# Função que para o robô a cada segundo, assegurando que ele não efetue movimentos desnecessários
def stop_robot():
   movement = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
   print("Stopping!")
   pub.publish(movement)
   rospy.sleep(0.5)

if __name__ == "__main__":
    rospy.init_node("trilha_quadrado")
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=3)

    try:
        while not rospy.is_shutdown():
            movement = None
            if i == 0 or i % 2 == 0:
                movement = Twist(Vector3(movements[i], 0, 0), Vector3(0, 0, 0))
                print("Moving!")
            else:
                movement = Twist(Vector3(0, 0, 0), Vector3(0, 0, movements[i]))
                print("Turning!")

            pub.publish(movement)
            
            if i != len(movements) - 1:
                i += 1
            else:
                i = 0
            
            rospy.sleep(1)
            stop_robot()  

    except rospy.ROSInterruptException:
        print("Ocorreu uma exceção com o rospy")