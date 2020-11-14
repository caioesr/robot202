#!/usr/bin/python
# -*- coding: utf-8 -*-

import cv2
import numpy as np

class center_mass:
    def __init__(self, low, high):
        self.low = low
        self.high = high

    def filter_color(self, bgr):
        """
        Retorna uma mascara filtrando a cor entre a range low e high
        """
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.low, self.high)
        return mask

    def center_coords(self, mask):
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

    def crosshair(self, img, point, size, color):
        """
        Desenha uma cruz no centro de massa
        """
        x,y = point
        cv2.line(img,(x - size,y),(x + size,y),color,5)
        cv2.line(img,(x,y - size),(x, y + size),color,5)

    def center_of_mass_region(self, mask, x1, y1, x2, y2):
        mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        clipped = mask[y1:y2, x1:x2]
        c = self.center_coords(clipped)
        c = list(c)
        c[0] += x1
        c[1] += y1
        c = tuple(c)
        self.crosshair(mask_bgr, c, 10, (0,0,255))
        cv2.rectangle(mask_bgr, (x1, y1), (x2, y2), (0,255,0),2,cv2.LINE_AA)
        return c, mask_bgr