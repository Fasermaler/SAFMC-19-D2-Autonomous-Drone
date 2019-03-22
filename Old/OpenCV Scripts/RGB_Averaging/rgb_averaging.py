#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Jan 30 15:18:35 2019

@author: fasermaler
"""

import cv2
import numpy as np
cap = cv2.VideoCapture(0)
while True:
    r, img = cap.read()
    #img = cv2.imread('arrow.png')
    #print(img)
    b_channel = np.array(img[:,:,0]).astype('float')
    g_channel = np.array(img[:,:,1]).astype('float')
    r_channel = np.array(img[:,:,2]).astype('float')
    cv2.imshow('b_chan', b_channel)
#    cv2.imshow('g_chan', g_channel)
#    cv2.imshow('r_chan', r_channel)
    bgr_channel = np.add((np.add(b_channel, g_channel)), r_channel)
    img_rec_red = np.divide(r_channel, bgr_channel)
    img_rec_red = img_rec_red * 255
    img_rec_red = np.floor(img_rec_red).astype('uint8')
    #cv2.imshow('bg_chan', bg_channel)
    #print(bgr_channel)
    #print(img_rec_red)
    cv2.imshow('preview', img_rec_red)
#    bg_channel = b_channel + g_channel
#    bgr_add_channel = np.add((np.add(b_channel, g_channel)), r_channel)
#    print(bg_channel)
#    np.floor(bgr_add_channel)
#    print(bgr_add_channel)
#    print(np.add([[255, 255]], [[255, 255]]))
#    img_perc_red = np.divide(r_channel, bgr_add_channel, out=np.zeros_like(r_channel), where=bgr_add_channel!=0)
#    #img_gray = cv2.cvtColor(img_perc_red, cv2.COLOR_BGR2GRAY)
#    img = cv2.bitwise_and(img, img, img_perc_red)
#    cv2.imshow('preview', img)
#    
    
    
    k = cv2.waitKey(1)
    if k == 0xFF & ord("q"):
        break
cv2.destroyAllWindows()
#cap.release()
