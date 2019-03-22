#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Jan 30 15:18:35 2019

@author: fasermaler
"""

import cv2
import numpy as np
import imutils
#cap = cv2.VideoCapture(0)
points = [None] * 4
t_points = [None] * 4
blacked_cells = [None] * 5
black_count = 0
blacked = False

def order_points(pts):
	# initialzie a list of coordinates that will be ordered
	# such that the first entry in the list is the top-left,
	# the second entry is the top-right, the third is the
	# bottom-right, and the fourth is the bottom-left
	rect = np.zeros((4, 2), dtype = "float32")
 
	# the top-left point will have the smallest sum, whereas
	# the bottom-right point will have the largest sum
	s = pts.sum(axis = 1)
	rect[0] = pts[np.argmin(s)]
	rect[2] = pts[np.argmax(s)]
 
	# now, compute the difference between the points, the
	# top-right point will have the smallest difference,
	# whereas the bottom-left will have the largest difference
	diff = np.diff(pts, axis = 1)
	rect[1] = pts[np.argmin(diff)]
	rect[3] = pts[np.argmax(diff)]
 
	# return the ordered coordinates
	return rect

def four_point_transform(image, pts):
	# obtain a consistent order of the points and unpack them
	# individually
	rect = order_points(pts)
	(tl, tr, br, bl) = rect
 
	# compute the width of the new image, which will be the
	# maximum distance between bottom-right and bottom-left
	# x-coordiates or the top-right and top-left x-coordinates
	widthA = np.sqrt(((br[0] - bl[0]) ** 2) + ((br[1] - bl[1]) ** 2))
	widthB = np.sqrt(((tr[0] - tl[0]) ** 2) + ((tr[1] - tl[1]) ** 2))
	maxWidth = max(int(widthA), int(widthB))
 
	# compute the height of the new image, which will be the
	# maximum distance between the top-right and bottom-right
	# y-coordinates or the top-left and bottom-left y-coordinates
	heightA = np.sqrt(((tr[0] - br[0]) ** 2) + ((tr[1] - br[1]) ** 2))
	heightB = np.sqrt(((tl[0] - bl[0]) ** 2) + ((tl[1] - bl[1]) ** 2))
	maxHeight = max(int(heightA), int(heightB))
 
	# now that we have the dimensions of the new image, construct
	# the set of destination points to obtain a "birds eye view",
	# (i.e. top-down view) of the image, again specifying points
	# in the top-left, top-right, bottom-right, and bottom-left
	# order
	dst = np.array([
		[0, 0],
		[maxWidth - 1, 0],
		[maxWidth - 1, maxHeight - 1],
		[0, maxHeight - 1]], dtype = "float32")
 
	# compute the perspective transform matrix and then apply it
	M = cv2.getPerspectiveTransform(rect, dst)
	warped = cv2.warpPerspective(image, M, (maxWidth, maxHeight))
 
	# return the warped image
	return warped

def display_rects(in_img, rects):
	"""Displays rectangles on the image."""
	img = in_img.copy()
	for rect in rects:
		img = cv2.rectangle(img, tuple(int(x) for x in rect[0]), tuple(int(x) for x in rect[1]), 255)
	cv2.imshow('previes', img)
	return img

while True:
    #r, img = cap.read()
    img = cv2.imread('SAFMC_grid_indicated.png')
    #img = cv2.imread('arrow.png')
    #print(img)
    b_channel = np.array(img[:,:,0]).astype('float')
    g_channel = np.array(img[:,:,1]).astype('float')
    r_channel = np.array(img[:,:,2]).astype('float')
    cv2.imshow('b_chan', b_channel)
#    cv2.imshow('g_chan', g_channel)
#    cv2.imshow('r_chan', r_channel)
    bgr_channel = np.add((np.add(b_channel, g_channel)), r_channel)
    img_rec_green = np.divide(g_channel, bgr_channel)
    np.floor(img_rec_green)
    img_rec_green = img_rec_green * 255
    img_rec_green = np.floor(img_rec_green).astype('uint8')
    #cv2.imshow('bg_chan', bg_channel)
    #print(bgr_channel)
    #print(img_rec_red)
    ret, th = cv2.threshold(img_rec_green,127,255,cv2.THRESH_BINARY)
    #cv2.imshow('preview', img_rec_green)
    # calculate moments of binary image
    im2, contours, hierarchy = cv2.findContours(th,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
#    try:
    count = 0
    for c in contours:
        # calculate moments for each contour
        M = cv2.moments(c)
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        points[count] = [cX, cY]
        t_points[count] = (cX, cY)
        cv2.circle(img, (cX, cY), 5, (0, 0, 0), -1)
        cv2.drawContours(img, [c], -1, (255, 100, 255), 2)
        count += 1
#    except:
#        print("no contours")
    print(points)
    pts = np.array(points, np.int32)
    pts = pts.reshape((-1,1,2))
    t_points = np.array(t_points, dtype=np.float32)
    warped = (four_point_transform(img, t_points))
    
    rect = cv2.boundingRect(pts)
    x,y,w,h = rect
    cropped = img[y:y+h, x:x+w].copy()
    #cv2.imshow('threshold', th)
    cv2.imshow('original', img)
    #cv2.imshow('cropped', cropped)
    print(t_points)
    
    # Do Thresholding on the cropped and warped image
    gray_warped = cv2.cvtColor(warped, cv2.COLOR_BGR2GRAY)
    blurred_warped = cv2.GaussianBlur(gray_warped.copy(), (9, 9), 0)
    ret, th2 = cv2.threshold(gray_warped, 127, 255, cv2.THRESH_BINARY)
    th2 = cv2.bitwise_not(th2, th2)
    kernel = np.ones((5,5),np.uint8)
    dilated_th2 = cv2.dilate(th2, kernel)
    #cv2.imshow('th2', th2)
    cv2.imshow('dilated_th2', dilated_th2)
    
    
    # Split image into squares
    squares = []
    h_side = dilated_th2.shape[0] / 5
    w_side = dilated_th2.shape[1] / 10
    for i in range(10):
        for j in range(5):
            p1 = (i * w_side, j * h_side)  # Top left corner of a bounding box
            p2 = ((i + 1) * w_side, (j + 1) * h_side)  # Bottom right corner of bounding bo
            squares.append((p1, p2))
	
    display_rects(warped, squares)
    
    # Check percentage of red pixels in each square
    RED_MIN1 = np.array([0, 100, 100], np.uint8)
    RED_MAX1 = np.array([10, 255, 255], np.uint8)
    RED_MIN2 = np.array([160, 100, 100], np.uint8)
    RED_MAX2 = np.array([179, 255, 255], np.uint8)
    
    cell_counter = 0 # Used to count which cell has had a red square
    print(blacked_cells)
    for square in squares:
        if cell_counter not in blacked_cells:
            perc_red = 0 
            cropped = warped[int(square[0][1]):int(square[0][1]+w_side), int(square[0][0]):int(square[0][0]+h_side)]
            #cv2.imshow("crop", cropped)
            
            total_px = cropped.shape[0] * cropped.shape[1]
            print(total_px)
            hsv_cropped = cv2.cvtColor(cropped, cv2.COLOR_BGR2HSV)
            
        
            dst1 = cv2.inRange(hsv_cropped, RED_MIN1, RED_MAX1)
            dst2 = cv2.inRange(hsv_cropped, RED_MIN2, RED_MAX2)
            no_red = cv2.countNonZero(dst1)
            no_red = no_red + cv2.countNonZero(dst2)
            print('The number of red pixels is: ' + str(no_red))
            perc_red = no_red / total_px
            if perc_red > 0.5:
                cv2.rectangle(warped, (int(square[0][0]), int(square[0][1])), (int(square[1][0]), int(square[1][1])), (0,255,0), 3)
                #blacked_cells[black_count] = cell_counter # Let's the program know that this cell has been detected before
                #black_count += 1
            
        else:
            pass            #print("The cell is: " + str(cell_counter) + " and I am running this")
            #if blacked:
                #cv2.rectangle(warped, (int(square[0][0]), int(square[0][1])), (int(square[1][0]), int(square[1][1])), (0,0,0), -1)
        cell_counter += 1
    cv2.imshow('transform', warped)
    
    #cv2.imshow('transform', (four_point_transform(img, t_points)))
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
    elif k == 0xFF & ord("k"):
        blacked = True
cv2.destroyAllWindows()
#cap.release()
