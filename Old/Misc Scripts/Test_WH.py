# -*- coding: utf-8 -*-
"""
Created on Mon Jan 28 00:04:15 2019

@author: imossim
"""

import cv2
import imutils
import numpy as np
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(hsv,(5,5),0)
    
    lower_red = np.array([170,150,100])
    upper_red = np.array([180,255,255])
    
    #mask = cv2.inRange(blur,lower_red, upper_red)
    ret1,thresh = cv2.threshold(blur,25,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C)
    
    cnts = cv2.findContours(thresh.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    
    for c in cnts:
        M = cv2.moments(c)
        try:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
        except:
            pass
        
        cv2.drawContours(frame, [c], -1, (0, 255, 0), 2)
        cv2.circle(frame, (cX, cY), 7, (255, 255, 255), -1)
        cv2.putText(frame, "center", (cX - 20, cY - 20),
        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
    
    cv2.imshow('frame',frame)
    cv2.imshow('blur,res',hsv)
    cv2.imshow('thresh',thresh)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()