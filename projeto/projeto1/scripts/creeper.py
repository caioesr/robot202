#! /usr/bin/env python
# -*- coding:utf-8 -*-
import cv2 
import numpy as np

class creeper:

    def __init__(self,color):
        if color[1] == "blue":
            self.low = np.array([90,50,50],dtype=np.uint8)
            self.high = np.array([100,255,255],dtype=np.uint8)
        elif color[1] == "green":
            self.low = np.array([50,50,50],dtype=np.uint8)
            self.high = np.array([60,255,255],dtype=np.uint8)
        else:
            self.low = np.array([140,50,50],dtype=np.uint8)
            self.high = np.array([150,255,255],dtype=np.uint8)

    
    def filtra_creeper(self,bgr):


        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.low, self.high)
        return mask

    def creeper_coords(self, mask):
        """
        Retorna uma tupla (x,y) com o centro de massa da mascara
        """
        M = cv2.moments(mask)
        try:
            x = int(M["m10"]/M["m00"])
            y = int(M["m01"]/M["m00"])
            return (x,y)
        except:
            return (0,0)

    def center_of_creeper_region(self, mask, x1, y1, x2, y2):
        mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        clipped = mask[y1:y2, x1:x2]
        c = self.creeper_coords(clipped)
        c = list(c)
        c[0] += x1
        c[1] += y1
        c = tuple(c)
        cv2.rectangle(mask_bgr, (x1, y1), (x2, y2), (255,0,0),2,cv2.LINE_AA)
        return mask_bgr