#!/usr/bin/python
# -*- coding: utf-8 -*-

import cv2
import numpy as np

from auxiliar import center_mass

cap = cv2.VideoCapture('line_following.mp4')

# Valores para amarelo usando um color picker
low = np.array([22, 50, 50],dtype=np.uint8)
high = np.array([36, 255, 255],dtype=np.uint8)

cm = center_mass(low, high)

while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()
    
    if ret == False:
        print("Codigo de retorno FALSO - problema para capturar o frame")

    # Our operations on the frame come here
    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB) 
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    
    cv2.imshow('frame',frame)
    mask = cm.filter_color(frame)
    mask_bgr = cm.center_of_mass_region(mask, 20, 200, frame.shape[1] - 20, frame.shape[0]-100) # Lembrando que negativos contam a partir do fim`
    cv2.imshow('mask', mask_bgr)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()