#! /usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
from geometry_msgs.msg import Twist, Vector3
import math
import time

v = 0.1  # Velocidade linear
w = math.pi/2  # Velocidade angular

if __name__ == "__main__":
    rospy.init_node("roda_exemplo")
    pub = rospy.Publisher("cmd_vel", Twist, queue_size=3)

    try:
        i = 0
        while not rospy.is_shutdown():
            vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
            if i%2:
                vel = Twist(Vector3(0,0,0), Vector3(0,0,w))
            else:
                vel = Twist(Vector3(v,0,0), Vector3(0,0,0))
            pub.publish(vel)
            i += 1
            rospy.sleep(0.1)
    except rospy.ROSInterruptException:
        print("Ocorreu uma exceção com o rospy")