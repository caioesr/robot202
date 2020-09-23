#! /usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
import numpy as np
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan

foward = True

def move_foward(dado):
    global foward
    print(np.array(dado.ranges).round(2))
    dado = list(np.array(dado.ranges))
    for i in dado[:15]:
        if i <= 1:
            foward = False
            return
        elif i <= 1.02:
            foward = True
            return
        else:
            foward = True
            return
    for i in dado[:345:-1]:
        if i <= 1:
            foward = False
            return
        elif i <= 1.02:
            foward = True
            return
        else:
            foward = True
            return

if __name__ == "__main__":
    rospy.init_node("indeciso")
    pub = rospy.Publisher("cmd_vel", Twist, queue_size=3)
    recebe_scan = rospy.Subscriber("/scan", LaserScan, move_foward)

    try:
        while not rospy.is_shutdown():
            vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
            if foward:
                vel = vel = Twist(Vector3(0.2,0,0), Vector3(0,0,0))
            else:
                vel = Twist(Vector3(-0.2,0,0), Vector3(0,0,0))
            pub.publish(vel)
            rospy.sleep(0.1)
    except rospy.ROSInterruptException:
        print("Ocorreu uma exceção com o rospy")