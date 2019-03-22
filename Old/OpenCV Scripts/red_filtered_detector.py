

from picamera.array import PiRGBArray
from picamera import PiCamera
import cv2
import numpy as np
import time
from fractions import Fraction
from PIL import Image


#cap = cv2.VideoCapture(0)
camera = PiCamera()
camera.resolution = (426, 240)
camera.framerate = 24
camera.exposure_mode = 'off'
camera.exposure_compensation = -3
camera.drc_strength = 'off'
camera.still_stats = False

camera.awb_mode = 'off'
camera.awb_gains = (Fraction(25, 16), Fraction(25,16))

rawCapture = PiRGBArray(camera, size=(426, 240))


# allow the camera to warmup
time.sleep(0.1)

# lower = [135, 130, 50]
# upper = [180, 200, 255]
# lower = [160, 100, 100]
# upper = [180, 255, 255]

# lower2 = [0, 100, 100]
# upper2 = [10, 255, 255]

#lower1 = [0, 50, 50]
#upper1 = [5, 255, 255]

out = cv2.VideoWriter(str(time.time()) + ".avi",cv2.VideoWriter_fourcc('M','J','P','G'), 10, (426, 240))

# lower = np.array(lower, dtype = "uint8")
# upper = np.array(upper, dtype = "uint8")

# lower2 = np.array(lower2, dtype = "uint8")
# upper2 = np.array(upper2, dtype = "uint8")

#lower1 = np.array(lower1, dtype = "uint8")
#upper1 = np.array(upper1, dtype = "uint8")

for img in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    #print(camera.awb_gains)
    #r, frame = cap.read()
    for i in range(5): # Clears the 5 frame buffer 
        frame = img.array
    height, width = frame.shape[:2]
    centre = (int(width/2), int(height/2))
    #frame = cv2.GaussianBlur(frame, (9, 9), 0)
    #frame = cv2.medianBlur(frame,3)
    #frame = cv2.GaussianBlur(frame, (9, 9), 0)
    #mask = cv2.inRange(frame, lower, upper)
    #mask2 = cv2.inRange(frame, lower2, upper2)
    #mask2 = cv2.inRange(frame, lower1, upper1)
    #mask = mask1 + mask2
    #img_rec_red = cv2.bitwise_and(frame, frame, mask = mask)
    #img_rec_redo = cv2.bitwise_and(frame, frame, mask = mask2)
    #cv2.imshow("pre or1", img_rec_red)
    #cv2.imshow("pre or2", img_rec_redo)
    #img_rec_red = cv2.bitwise_or(img_rec_red, img_rec_redo)
    b_channel = np.array(frame[:,:,0]).astype('float')
    g_channel = np.array(frame[:,:,1]).astype('float')
    r_channel = np.array(frame[:,:,2]).astype('float')
#     #cv2.imshow('b_chan', b_channel)
# #    cv2.imshow('g_chan', g_channel)
# #    cv2.imshow('r_chan', r_channel)
    bgr_channel = np.add((np.add(b_channel, g_channel)), r_channel)
    img_rec_red2 = np.subtract(r_channel,((b_channel + g_channel)/ 2))
    #img_rec_red2 = np.divide(r_channel, 255)
    img_rec_red2 = np.divide(img_rec_red2,255) 
    #img_rec_red2 = np.square(img_rec_red2)
    img_rec_red2[img_rec_red2 < 0.3] = 0
    img_rec_red2 = img_rec_red2 * 255
    img_rec_red2 = np.floor(img_rec_red2).astype('uint8')
    #img_rec_red = cv2.cvtColor(img_rec_red, cv2.COLOR_BGR2GRAY)
    #cv2.imshow('recred2', img_rec_red2)

    ret, th = cv2.threshold(img_rec_red2,10,255,cv2.THRESH_BINARY)
    
    
    #ret, th = cv2.threshold(r_channel.astype('uint8'),110,255,cv2.THRESH_BINARY)
    #th = cv2.bitwise_not(th, th)
    kernel = np.ones((5,5),np.uint8)
    #th = cv2.erode(th, kernel)
    th = cv2.dilate(th, kernel)
    th = cv2.GaussianBlur(th, (5,5), 0)
    try:
        M = cv2.moments(th)

        # calculate x,y coordinate of center
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
         
        # put text and highlight the center
        cv2.circle(frame, (cX, cY), 5, (255, 255, 255), -1)
        #cv2.putText(frame, "centroid", (cX - 25, cY - 25),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        cv2.line(frame, centre, (cX, cY), (255,0,0), 2)
        dX = cX - centre[0] 
        dY = centre[1] - cY
        cv2.putText(frame, ("(" + str(dX) + ", " + str(dY) + " )"), (centre[0] - 20, centre[1] - 20),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

        print('Velocities: ' + str(dX) + "," + str(dY))
    except:
        print("No centre detected")

    #kernel2 = np.ones((15,15),np.uint8)
    #eroded_th = cv2.erode(dilated_th, kernel2)
    #blurred_th = cv2.GaussianBlur(eroded_th.copy(), (9, 9), 0)
    #eroded_th = cv2.bitwise_not(eroded_th,eroded_th)
    #dilated_th = cv2.bitwise_not(dilated_th, dilated_th)


    
    
    # circles = cv2.HoughCircles(th,cv2.HOUGH_GRADIENT, 1,1000,
    #                             param1=40,param2=23,minRadius=20,maxRadius=0)
    # try:
    #     circles = np.uint16(np.around(circles))
    #     for i in circles[0,:]:
    #         # draw the outer circle
    #         cv2.circle(frame,(i[0],i[1]),i[2],(0,255,0),2)
    #         # draw the center of the circle
    #         cv2.circle(frame,(i[0],i[1]),2,(0,0,255),3)
    # except:
    #     pass
    
    cv2.imshow('original', frame)
    #cv2.imshow('rec_red',img_rec_red)
    cv2.imshow('detected circles',th)
    out.write(frame)
    k = cv2.waitKey(1)
    rawCapture.truncate(0)
    if k == 0xFF & ord("q"):
        break
#cv2.destroyAllWindows()
#cap.release()

out.release()