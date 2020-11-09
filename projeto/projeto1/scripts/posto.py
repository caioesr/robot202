#! /usr/bin/env python
# -*- coding:utf-8 -*-

import cv2
import numpy as np
from geometry_msgs.msg import Twist, Vector3
from claw import claw

class posto:

    def __init__(self, results, relevant_class, v, w, claw):
        self.relevant_class = relevant_class
        self.vel_rot = w
        self.vel_trans = v
        self.result = None
        for r in results:
            if (r[0] == relevant_class and r[1] > 90):
                self.result = r
        self.diff = None
        self.claw = claw

    def check_result(self):
        if self.result != None:
            return True
        return False

    def get_diff(self, frame):
        x = int(frame.shape[1]/2)
        if(self.check_result()):
            image_center_x = (self.result[2][1] + self.result[3][1])/2
            self.diff = (image_center_x - x)
        else:
            self.diff = None

    def get_velocity(self, frame):
        if(self.claw.get_arm_state >= 0 and self.check_result()):
            self.get_diff(frame)
            if(self.diff > 0):
                if(self.diff < 10):
                    return Twist(Vector3(self.vel_trans,0,0), Vector3(0,0,0))
                else:
                    return Twist(Vector3(0,0,0), Vector3(0,0,self.vel_rot))
            else:
                if(self.diff > -10):
                    return Twist(Vector3(self.vel_trans,0,0), Vector3(0,0,0))
                else:
                    return Twist(Vector3(0,0,0), Vector3(0,0,-self.vel_rot))
        return None

    def turn_back(self):
        pass