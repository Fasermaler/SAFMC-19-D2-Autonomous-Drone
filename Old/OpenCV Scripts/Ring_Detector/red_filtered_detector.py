

from picamera.array import PiRGBArray
from picamera import PiCamera
import cv2
import numpy as np
import time
from PIL import Image


#cap = cv2.VideoCapture(0)
camera = PiCamera()
camera.resolution = (426, 240)
camera.framerate = 90
camera.exposure_mode = 'fixedfps'
camera.exposure_compensation = -5
rawCapture = PiRGBArray(camera, size=(426, 240))

# allow the camera to warmup
time.sleep(0.1)

for cap in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    #r, frame = cap.read()
    for i in range(5): # Clears the 5 frame buffer 
        frame = cap.array
    
    b_channel = np.array(frame[:,:,0]).astype('float')
    g_channel = np.array(frame[:,:,1]).astype('float')
    r_channel = np.array(frame[:,:,2]).astype('float')
    #cv2.imshow('b_chan', b_channel)
#    cv2.imshow('g_chan', g_channel)
#    cv2.imshow('r_chan', r_channel)
    bgr_channel = np.add((np.add(b_channel, g_channel)), r_channel)
    img_rec_red = np.divide(r_channel, bgr_channel)
    img_rec_red = img_rec_red * 255
    img_rec_red = np.floor(img_rec_red).astype('uint8')
    ret, th = cv2.threshold(img_rec_red,110,255,cv2.THRESH_BINARY)
    #ret, th = cv2.threshold(r_channel.astype('uint8'),110,255,cv2.THRESH_BINARY)
    th = cv2.bitwise_not(th, th)
    kernel = np.ones((19,19),np.uint8)
    dilated_th = cv2.dilate(th, kernel)
    kernel2 = np.ones((15,15),np.uint8)
    eroded_th = cv2.erode(dilated_th, kernel2)
    blurred_th = cv2.GaussianBlur(eroded_th.copy(), (9, 9), 0)
    #eroded_th = cv2.bitwise_not(eroded_th,eroded_th)
    #dilated_th = cv2.bitwise_not(dilated_th, dilated_th)


    
    
    circles = cv2.HoughCircles(blurred_th,cv2.HOUGH_GRADIENT, 1,1000,
                                param1=50,param2=20,minRadius=80,maxRadius=0)
    try:
        circles = np.uint16(np.around(circles))
        for i in circles[0,:]:
            # draw the outer circle
            cv2.circle(frame,(i[0],i[1]),i[2],(0,255,0),2)
            # draw the center of the circle
            cv2.circle(frame,(i[0],i[1]),2,(0,0,255),3)
    except:
        pass
    
    cv2.imshow('original', frame)
    cv2.imshow('detected circles',blurred_th)
    k = cv2.waitKey(1)
    rawCapture.truncate(0)
    if k == 0xFF & ord("q"):
        break
cv2.destroyAllWindows()
#cap.release()

