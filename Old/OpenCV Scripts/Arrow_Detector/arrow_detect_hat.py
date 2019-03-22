#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Jan 30 17:51:52 2019

@author: fasermaler
"""

import cv2
import numpy as np
from collections import defaultdict
#cap = cv2.VideoCapture(0)

while True:
    #r, img = cap.read()
    print('program start')
    img = cv2.imread('arrow.png')
    cv2.imshow('original', img)
    b_channel = np.array(img[:,:,0]).astype('float')
    g_channel = np.array(img[:,:,1]).astype('float')
    r_channel = np.array(img[:,:,2]).astype('float')
    #cv2.imshow('b_chan', b_channel)
    #cv2.imshow('g_chan', g_channel)
    #cv2.imshow('r_chan', r_channel)
    bgr_channel = np.add((np.add(b_channel, g_channel)), r_channel)
    img_rec_red = np.divide(r_channel, bgr_channel)
    img_rec_red = img_rec_red * 255
    img_rec_red = np.floor(img_rec_red).astype('uint8')
    
    #gray = cv2.cvtColor(img_rec_red,cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(img_rec_red,50,150,apertureSize = 3)
    
    lines = cv2.HoughLines(edges,1,np.pi/40,40)
    
    # convert the grayscale image to binary image
    ret,thresh = cv2.threshold(img_rec_red,127,255,0)
    cv2.imshow('thresh', thresh)
     
    # calculate moments of binary image
    im2, contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    try:
        for c in contours:
            # calculate moments for each contour
            M = cv2.moments(c)
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            cv2.drawContours(img, [c], -1, (255, 100, 255), 2)
            cv2.circle(img, (cX, cY), 5, (0, 0, 0), -1)
            cv2.putText(img, "m_centroid", (cX + 15, cY + 15),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
            
            
        kernel = cv2.getStructuringElement(cv2.MORPH_CROSS,(19,19))
        #erosion = cv2.erode(img,kernel,iterations = 1)
        tophat = cv2.morphologyEx(img_rec_red, cv2.MORPH_TOPHAT, kernel)
        cv2.imshow('tophat', tophat)
        blackhat = cv2.morphologyEx(img_rec_red, cv2.MORPH_BLACKHAT, kernel)
        
        
        im2_2, bh_contours, hierarchy_2 = cv2.findContours(blackhat,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        try:
            for c in bh_contours:
                # calculate moments for each contour
                BM = cv2.moments(c)
                bX = int(BM["m10"] / BM["m00"])
                bY = int(BM["m01"] / BM["m00"])
                cv2.circle(img, (bX, bY), 5, (255, 0, 0), -1)
                cv2.putText(img, "b_cen", (bX + 15, bY + 15),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
                cv2.drawContours(blackhat, [c], -1, (255, 100, 255), 2)
                if bX > cX:
                    cv2.putText(img, "GO RIGHT", (cX - 50, cY - 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                elif bX < cX:
                    cv2.putText(img, "GO LEFT", (cX - 50, cY - 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)   
                
            
                
        except:
            pass
    except:
        print("no contours")   

    
    
    cv2.imshow('blackhat', blackhat)
    cv2.imshow('preview', img)
    cv2.imshow('rec_red', img_rec_red)

    
    k = cv2.waitKey(1)
    if k == 0xFF & ord("q"):
        break
cv2.destroyAllWindows()
#cap.release()
