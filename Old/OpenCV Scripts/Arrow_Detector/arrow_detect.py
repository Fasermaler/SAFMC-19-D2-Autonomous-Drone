#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Jan 30 17:51:52 2019

@author: fasermaler
"""


from picamera.array import PiRGBArray
from picamera import PiCamera
import cv2
import numpy as np
import time
import random 
from fractions import Fraction
from PIL import Image
from math import cos
from sympy import Point, Polygon, pi

#cap = cv2.VideoCapture(0)


camera = PiCamera()
camera.resolution = (426, 240)
camera.framerate = 24
camera.exposure_mode = 'auto'
camera.exposure_compensation = -3
camera.drc_strength = 'off'
camera.still_stats = False


camera.awb_mode = 'off'
camera.awb_gains = (Fraction(167, 103), Fraction(27,16))

rawCapture = PiRGBArray(camera, size=(426, 240))

for img in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):

    for i in range(5): # Clears the 5 frame buffer 
        frame = img.array
    height, width = frame.shape[:2]
    centre = (int(width/2), int(height/2))

    b_channel = np.array(frame[:,:,0]).astype('float')
    g_channel = np.array(frame[:,:,1]).astype('float')
    r_channel = np.array(frame[:,:,2]).astype('float')

    bgr_channel = np.add((np.add(b_channel, g_channel)), r_channel)
    img_rec_red2 = np.subtract(r_channel,((b_channel + g_channel)/ 2))
    #img_rec_red2 = np.divide(r_channel, 255)
    img_rec_red2 = np.divide(img_rec_red2,255) 
    #img_rec_red2 = np.square(img_rec_red2)
    img_rec_red2[img_rec_red2 < 0.3] = 0
    img_rec_red2 = img_rec_red2 * 255
    #img_rec_red = np.floor(img_rec_red2).astype('uint8')

    try:
        #gray = cv2.cvtColor(img_rec_red,cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(img_rec_red2,50,150,apertureSize = 3)
        cv2.imshow("edges", edges)
        lines = cv2.HoughLines(edges,1,np.pi/40,40)
        
        # convert the grayscale image to binary image
        #ret,thresh = cv2.threshold(img_rec_red,127,255,0)
        #cv2.imshow('thresh', thresh)
         
        # calculate moments of binary image
        im2, contours, hierarchy = cv2.findContours(img_rec_red2,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        for c in contours:
            # calculate moments for each contour
            M = cv2.moments(c)
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            cv2.circle(img, (cX, cY), 5, (255, 100, 255), -1)
            cv2.putText(img, "centroid", (cX - 25, cY - 25),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            cv2.drawContours(img, [c], -1, (255, 100, 255), 2)
            x,y,w,h = cv2.boundingRect(c)
            cv2.rectangle(img,(x,y),(x+w,y+h),(0,255,0),2)
            rX = int(x+(w/2))
            rY = int(y+(h/2))
            cv2.circle(img, (rX, rY), 5, (0,255,255), -1)
            
            
            

        
        
        for line in lines:
            print(line)
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
                cv2.line(img_rec_red,(x1,y1),(x2,y2),(0,255,0),2)
                
        
        cv2.putText(img, "centroid", (cX - 25, cY - 25),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
        left_discrepancy = rY - cY
        print(left_discrepancy)
        if left_discrepancy > 0:
            cv2.putText(img, "GO LEFT", (rX - 50, rY - 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        elif left_discrepancy < 0:
            cv2.putText(img, "GO RIGHT", (rX - 50, rY - 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        cv2.putText(img, "centroid", (cX - 25, cY - 25),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
    except:
        pass
    cv2.imshow('preview', frame)
    cv2.imshow('rec_red', img_rec_red2)

    
    k = cv2.waitKey(1)
    rawCapture.truncate(0)
    if k == 0xFF & ord("q"):
        break
cv2.destroyAllWindows()