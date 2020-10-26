#! /usr/bin/env python
# -*- coding:utf-8 -*-

import cv2
import numpy as np
from geometry_msgs.msg import Twist, Vector3

class tracker:
    def __init__(self, v, w):
        self.vel_rot = w
        self.vel_trans = v

    def get_center(self, frame):
        x = int(frame.shape[1]/2)
        y = int(frame.shape[0]/2)

        return (x,y)

    def get_diff(self, point, cm):
        x,y = point
        x_cm,y_cm = cm
        diff = x - x_cm
        return diff

    def crosshair(self, img, point, size, color):
        """
        Desenha uma cruz no centro do frame
        """
        x,y = point
        cv2.line(img,(x - size,y),(x + size,y),color,5)
        cv2.line(img,(x,y - size),(x, y + size),color,5)

    def get_velocity(self, point, cm):
        diff = self.get_diff(point, cm)
        print(diff)
        if(diff > 0):
            if(diff < 10):
                return Twist(Vector3(self.vel_trans,0,0), Vector3(0,0,0))
            else:
                return Twist(Vector3(0,0,0), Vector3(0,0,self.vel_rot))
        else:
            if(diff > -10):
                return Twist(Vector3(self.vel_trans,0,0), Vector3(0,0,0))
            else:
                return Twist(Vector3(0,0,0), Vector3(0,0,-self.vel_rot))