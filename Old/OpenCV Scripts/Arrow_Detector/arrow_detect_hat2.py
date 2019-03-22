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
    img_rec_red2 = np.floor(img_rec_red2).astype('uint8')



    minimumArea = 30
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
            if cv2.contourArea(c) > minimumArea:
                # calculate moments for each contour
                M = cv2.moments(c)
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                cv2.circle(frame, (cX, cY), 5, (255, 100, 255), -1)
                cv2.putText(frame, "centroid", (cX - 25, cY - 25),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                cv2.drawContours(frame, [c], -1, (255, 100, 255), 2)
                x,y,w,h = cv2.boundingRect(c)
                cv2.rectangle(frame,(x,y),(x+w,y+h),(0,255,0),2)
                rX = int(x+(w/2))
                rY = int(y+(h/2))
                cv2.circle(frame, (rX, rY), 5, (0,255,255), -1)
            
            
            

        
        
            
        kernel = cv2.getStructuringElement(cv2.MORPH_CROSS,(19,19))
        #erosion = cv2.erode(img,kernel,iterations = 1)
        tophat = cv2.morphologyEx(img_rec_red2, cv2.MORPH_TOPHAT, kernel)
        cv2.imshow('tophat', tophat)
        blackhat = cv2.morphologyEx(img_rec_red2, cv2.MORPH_BLACKHAT, kernel)
        
        
        im2_2, bh_contours, hierarchy_2 = cv2.findContours(blackhat,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        try:
            for c in bh_contours:
                # calculate moments for each contour
                BM = cv2.moments(c)
                bX = int(BM["m10"] / BM["m00"])
                bY = int(BM["m01"] / BM["m00"])
                cv2.circle(frame, (bX, bY), 5, (255, 0, 0), -1)
                cv2.putText(frame, "b_cen", (bX + 15, bY + 15),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
                cv2.drawContours(blackhat, [c], -1, (255, 100, 255), 2)
                if bX > cX:
                    cv2.putText(frame, "GO RIGHT", (cX - 50, cY - 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                elif bX < cX:
                    cv2.putText(frame, "GO LEFT", (cX - 50, cY - 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)   
                
            
                
        except:
            pass
    except:
        print("no contours")   

    
    
    cv2.imshow('blackhat', blackhat)
    cv2.imshow('preview', frame)
    cv2.imshow('rec_red', img_rec_red2)
    k = cv2.waitKey(1)
    rawCapture.truncate(0)
    if k == 0xFF & ord("q"):
        break
cv2.destroyAllWindows()