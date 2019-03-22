
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
#from matplotlib import pyplot as plt
font = cv2.FONT_HERSHEY_SIMPLEX
for img in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    for i in range(5): # Clears the 5 frame buffer 
        frame = img.array
        #frame = cv.flip(frame,0)

    b_channel = np.array(frame[:,:,0]).astype('float')
    g_channel = np.array(frame[:,:,1]).astype('float')
    r_channel = np.array(frame[:,:,2]).astype('float')

    bgr_channel = np.add((np.add(b_channel, g_channel)), r_channel)
    img_rec_red2 = np.subtract(r_channel,((b_channel + g_channel)/ 2))
    #img_rec_red2 = np.divide(r_channel, 255)
    img_rec_red2 = np.divide(img_rec_red2,255) 
    #img_rec_red2 = np.square(img_rec_red2)
    img_rec_red2[img_rec_red2 < 0.28] = 0
    img_rec_red2 = img_rec_red2 * 255
    img_rec_red2 = np.floor(img_rec_red2).astype('uint8')
    img2 = cv2.imread('arrow.png',0) # trainImage
    # Initiate SIFT detector
    sift1 = cv2.xfeatures2d.SIFT_create(1000)
    sift2 = cv2.xfeatures2d.SIFT_create(1000)
        # define range of white color in HSV
        # change it according to your need !
    lower_white = np.array([0,0,100], dtype=np.uint8)
    upper_white = np.array([0,0,255], dtype=np.uint8)


	# find the keypoints and descriptors with SIFT
    kp1, des1 = sift1.detectAndCompute(img_rec_red2,None)
    kp2, des2 = sift2.detectAndCompute(img2,None)
    # FLANN parameters
    FLANN_INDEX_KDTREE = 1
    index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 2)
    search_params = dict(checks=100)   # or pass empty dictionary
    flann = cv2.FlannBasedMatcher(index_params,search_params)
    matches = flann.knnMatch(des1,des2,k=2)
    # Need to draw only good matches, so create a mask
    matchesMask = [[0,0] for i in xrange(len(matches))]
    # ratio test as per Lowe's paper
    matched_f = 0
    for i,(m,n) in enumerate(matches):
        if m.distance < 0.62*n.distance:
            matched_f += 1
            matchesMask[i]=[1,0]
    draw_params = dict(matchColor = (0,255,0),
                   singlePointColor = (255,0,0),
                   matchesMask = matchesMask,
                   flags = 0)
    img3 = cv2.drawMatchesKnn(img_rec_red2,kp1,img2,kp2,matches,None,**draw_params)
        #cv.imshow('frame',frame)
        #cv.imshow('mask',mask)
        #cv.imshow('res',res)
    cv2.imshow('image',img3)
    rawCapture.truncate(0)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
