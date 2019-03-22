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
def segment_by_angle_kmeans(lines, k=4, **kwargs):
    """Groups lines based on angle with k-means.

    Uses k-means on the coordinates of the angle on the unit circle 
    to segment `k` angles inside `lines`.
    """

    # Define criteria = (type, max_iter, epsilon)
    default_criteria_type = cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER
    criteria = kwargs.get('criteria', (default_criteria_type, 10, 1.0))
    flags = kwargs.get('flags', cv2.KMEANS_PP_CENTERS)
    attempts = kwargs.get('attempts', 1)

    # returns angles in [0, pi] in radians
    angles = np.array([line[0][1] for line in lines])
    # multiply the angles by two and find coordinates of that angle
    pts = np.array([[np.cos(2*angle), np.sin(2*angle)]
                    for angle in angles], dtype=np.float32)

    # run kmeans on the coords
    labels, centers = cv2.kmeans(pts, k, None, criteria, attempts, flags)[1:]
    labels = labels.reshape(-1)  # transpose to row vec

    # segment lines based on their kmeans label
    segmented = defaultdict(list)
    for i, line in zip(range(len(lines)), lines):
        segmented[labels[i]].append(line)
    segmented = list(segmented.values())
    return segmented

while True:
    #r, img = cap.read()
    print('program start')
    img = cv2.imread('arrow_right.png')
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
    print("raw lines:")
    print(lines)
    
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
            cv2.circle(img, (cX, cY), 5, (0, 0, 0), -1)
            cv2.putText(img, "centroid", (cX - 25, cY - 25),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
            cv2.drawContours(img, [c], -1, (255, 100, 255), 2)
    except:
        print("no contours")

    print('I want my lines')
    segmented = segment_by_angle_kmeans(lines, 4)

    try:
        for line in segmented[0]:
            print("group 0 lines:")
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
        for line in segmented[1]:
            print("group 1 lines:")
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
            
                cv2.line(img,(x1,y1),(x2,y2),(255,0,0),2)
                cv2.line(img_rec_red,(x1,y1),(x2,y2),(255,0,0),2)
        for line in segmented[2]:
            print("group 2 lines:")
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
            
                cv2.line(img,(x1,y1),(x2,y2),(0,255,100),2)
                cv2.line(img_rec_red,(x1,y1),(x2,y2),(0,255,100),2)
        for line in segmented[3]:
            print("group 3 lines:")
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
            
                cv2.line(img,(x1,y1),(x2,y2),(255,100,0),2)
                cv2.line(img_rec_red,(x1,y1),(x2,y2),(255,100,0),2)
            
    except:
        pass
    

    cv2.imshow('preview', img)
    cv2.imshow('rec_red', img_rec_red)

    
    k = cv2.waitKey(100)
    if k == 0xFF & ord("q"):
        break
cv2.destroyAllWindows()
#cap.release()
