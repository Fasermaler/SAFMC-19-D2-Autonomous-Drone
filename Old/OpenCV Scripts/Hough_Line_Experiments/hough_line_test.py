#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Jan 29 22:26:27 2019

@author: fasermalerstr(
"""
    

import cv2
import numpy as np
#cap = cv2.VideoCapture(0)
while True:
    #r, img = cap.read()
    img = cv2.imread('arrow.png')
    b_channel, g_channel, r_channel = cv2.split(img)
    bg_add_channel = np.add(b_channel, g_channel)
    bgr_add_channel = np.add(bg_add_channel, r_channel)
    img_perc_red = np.divide(r_channel, bgr_add_channel)
    
    img = cv2.bitwise_and(img, img, img_perc_red)
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray,50,150,apertureSize = 3)
    
    lines = cv2.HoughLines(edges,1,np.pi/40,40)
    print('start')
    print(lines)
    print('end')
    
    # convert the grayscale image to binary image
    ret,thresh = cv2.threshold(gray,127,255,0)
     
    # calculate moments of binary image
    M = cv2.moments(thresh)
     
    # calculate x,y coordinate of center
    cX = int(M["m10"] / M["m00"])
    cY = int(M["m01"] / M["m00"])
     
    # put text and highlight the center
    cv2.circle(img, (cX, cY), 5, (0, 0, 0), -1)
    
    try:
        for line in lines:
            for rho,theta in line:
                a = np.cos(theta)
                b = np.sin(theta)
                x0 = a*rho
                y0 = b*rho
                x1 = int(x0 + 1000*(-b))
                y1 = int(y0 + 1000*(a))
                x2 = int(x0 - 1000*(-b))
                y2 = int(y0 - 1000*(a))
            
                cv2.line(img,(x1,y1),(x2,y2),(0,255,0),2)
            
    except:
        pass
    cv2.putText(img, "centroid", (cX - 25, cY - 25),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
    cv2.imshow('preview', img)
    k = cv2.waitKey(1)
    if k == 0xFF & ord("q"):
        break
cv2.destroyAllWindows()
cap.release()
